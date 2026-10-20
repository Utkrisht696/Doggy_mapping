"""Causal scalar height fusion; deliberately has no X/Y/orientation state."""
from collections import deque
import math


class HeightFusion:
    def __init__(self, std=.01, process_variance_per_s=.0025, max_age=.05):
        if not all(math.isfinite(v) and v > 0 for v in (std, process_variance_per_s, max_age)):
            raise ValueError('Positive finite height-filter settings required')
        self.std, self.q, self.max_age = std, process_variance_per_s, max_age
        self.samples = deque(maxlen=5000)
        self.last_source = self.last_used = self.last_state = 0
        self.last_good_source = None
        self.previous_raw = self.z = self.offset = None
        self.variance = std**2
        self.counts = dict(received=0, invalid=0, old=0, source_jump=0,
                           anchored=0, updated=0, stale=0, missing=0, gated=0)

    def add(self, stamp, z):
        self.counts['received'] += 1
        if stamp <= 0 or not math.isfinite(z):
            self.counts['invalid'] += 1; return False
        if stamp <= self.last_source:
            self.counts['old'] += 1; return False
        self.last_source = stamp
        if self.last_good_source:
            previous_stamp, previous_z = self.last_good_source
            # A source-health gate, separate from disagreement with drifting LIO.
            if abs(z-previous_z) > .02+(stamp-previous_stamp)/1e9:
                self.counts['source_jump'] += 1; return False
        self.last_good_source = stamp, z
        self.samples.append((stamp, z))
        return True

    def step(self, stamp, raw_z):
        if stamp <= self.last_state or not math.isfinite(raw_z):
            raise ValueError('Invalid/nonincreasing LIO state')
        dt = (stamp-self.last_state)/1e9 if self.last_state else 0.
        dz = raw_z-self.previous_raw if self.previous_raw is not None else 0.
        self.last_state, self.previous_raw = stamp, raw_z
        self.z = raw_z if self.z is None else self.z+dz
        self.variance += self.q*dt
        while len(self.samples) > 1 and self.samples[1][0] <= stamp:
            self.samples.popleft()
        state = dict(z=self.z, variance=self.variance, source_stamp=None,
                     source_age=None, gain=0., innovation=None, mode='prediction_only')
        if not self.samples or self.samples[0][0] > stamp:
            self.counts['missing'] += 1
            return state
        source_stamp, height = self.samples[0]
        age = (stamp-source_stamp)/1e9
        state.update(source_stamp=source_stamp, source_age=age)
        if age > self.max_age or source_stamp <= self.last_used:
            self.counts['stale'] += 1
            return state
        self.last_used = source_stamp
        if self.offset is None:
            self.offset = self.z-height
            self.counts['anchored'] += 1
            state['mode'] = 'initial_alignment'
            return state
        target = height+self.offset
        residual = target-self.z
        variance = self.std**2+(.5*age)**2
        innovation_variance = self.variance+variance
        state['innovation'] = residual
        if abs(residual) > .5 or residual**2 > 64*innovation_variance:
            self.counts['gated'] += 1
            return state
        gain = self.variance/innovation_variance
        self.z += gain*residual
        self.variance = (1-gain)**2*self.variance+gain**2*variance
        self.counts['updated'] += 1
        state.update(z=self.z, variance=self.variance, gain=gain, mode='fused')
        return state
