"""Causality, missing measurements and measured-height preservation."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from height_fusion import HeightFusion


class HeightTests(unittest.TestCase):
    def test_missing_future_stale_and_invalid_source(self):
        f = HeightFusion(); t = 1792322606503946542
        self.assertEqual(f.step(t, 0.)['z'], 0.)
        f.add(t+100000000, .35)
        self.assertEqual(f.step(t+50000000, .01)['z'], .01)
        self.assertEqual(f.step(t+100000000, .02)['mode'], 'initial_alignment')
        self.assertFalse(f.add(t+100000000, .35))
        self.assertFalse(f.add(t+1, .35))
        self.assertFalse(f.add(t+200000000, math.nan))
        self.assertEqual(f.step(t+200000000, .03)['mode'], 'prediction_only')
        self.assertFalse(f.add(t+210000000, 1.35))
        with self.assertRaises(ValueError):
            f.step(t+200000000, .03)

    def test_constant_reference_rejects_lio_height_drift(self):
        f = HeightFusion(); t = 1792322606503946542
        for i in range(1000):
            stamp = t+i*65000000; f.add(stamp, .35)
            state = f.step(stamp, i*.003)
            self.assertLess(abs(state['z']), .005)
            self.assertGreater(state['variance'], 0)
        self.assertEqual(f.counts['gated'], 0)

    def test_preserve_ramp_if_reference_measures_world_elevation(self):
        f = HeightFusion(); t = 1792322606503946542
        for i in range(100):
            stamp = t+i*65000000; height = .003*i
            f.add(stamp, .35+height)
            self.assertLess(abs(f.step(stamp, height+.001*i)['z']-height), .003)

    def test_missing_reference_uses_lio_increments_and_grows_uncertainty(self):
        f = HeightFusion(); t = 1792322606503946542
        f.add(t, .35); f.step(t, 0.); initial = f.variance
        for i in range(1, 100):
            self.assertAlmostEqual(f.step(t+i*65000000, .001*i)['z'], .001*i)
        self.assertGreater(f.variance, initial)
