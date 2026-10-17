"""Observed-ground 2.5D mapping, independent of ROS and robot controls.

Only repeated 3D returns establish evidence. Empty space under a ray is not
evidence of supported ground. Terrain is estimated over neighborhoods larger
than the display cells; all classification limits are provisional parameters.
"""
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import json
import math

import numpy as np


@dataclass
class TerrainConfig:
    resolution: float = 0.05
    vertical_resolution: float = 0.03
    terrain_cell_size: float = 0.25
    min_observations: int = 2
    sensor_range: float = 10.0
    max_extent: float = 80.0
    max_voxels: int = 600000
    ground_tolerance: float = 0.08
    obstacle_min_height: float = 0.15
    obstacle_max_height: float = 1.0
    slope_cost_start_deg: float = 8.0
    slope_block_deg: float = 20.0
    roughness_cost_start: float = 0.02
    roughness_block: float = 0.06
    step_height: float = 0.12
    ground_connect_slope_deg: float = 40.0
    seed_min_below_imu: float = 0.15
    seed_max_below_imu: float = 0.85
    seed_radius: float = 1.25
    fit_radius: int = 2
    fit_min_cells: int = 6

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('min_observations','max_voxels','fit_radius','fit_min_cells'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f'{name} must be an integer')
        ratio = self.terrain_cell_size/self.resolution
        if not math.isclose(ratio, round(ratio)) or ratio < 1:
            raise ValueError('terrain_cell_size must be an integer multiple of resolution')
        if not (self.obstacle_min_height > self.ground_tolerance
                and self.obstacle_max_height > self.obstacle_min_height
                and self.slope_cost_start_deg < self.slope_block_deg < 90
                and self.roughness_cost_start < self.roughness_block
                and self.seed_min_below_imu < self.seed_max_below_imu):
            raise ValueError('Inconsistent terrain thresholds')
        if self.max_extent/self.resolution > 2000:
            raise ValueError('Bound the output grid to at most 2000 cells per side')


def shifted(array, dy, dx, fill=np.nan):
    """At [y,x], return the source [y+dy,x+dx], with bounded edges."""
    result = np.full(array.shape, fill, dtype=float)
    h, w = array.shape
    if abs(dy) >= h or abs(dx) >= w:
        return result
    y0, y1 = max(0,-dy), min(h,h-dy)
    x0, x1 = max(0,-dx), min(w,w-dx)
    result[y0:y1,x0:x1] = array[y0+dy:y1+dy,x0+dx:x1+dx]
    return result


def fit_ground(surface, cfg):
    """Local least-squares planes, followed by one residual rejection pass."""
    h, w = surface.shape
    model = None
    for iteration in range(2):
        sums = np.zeros((10,h,w), dtype=float)
        for dy in range(-cfg.fit_radius, cfg.fit_radius+1):
            for dx in range(-cfg.fit_radius, cfg.fit_radius+1):
                z = shifted(surface,dy,dx)
                valid = np.isfinite(z)
                x, y = dx*cfg.terrain_cell_size, dy*cfg.terrain_cell_size
                if model is not None:
                    prediction = model[...,0]*x+model[...,1]*y+model[...,2]
                    valid &= (~np.isfinite(prediction) | (np.abs(z-prediction) <= cfg.ground_tolerance))
                zz = np.where(valid,z,0.0)
                v = valid.astype(float)
                for index, value in enumerate((v,v*x,v*y,v*x*x,v*x*y,v*y*y,zz,zz*x,zz*y,zz*zz)):
                    sums[index] += value
        n,sx,sy,sxx,sxy,syy,sz,sxz,syz,szz = sums
        matrix = np.zeros((h,w,3,3))
        matrix[...,0,0],matrix[...,0,1],matrix[...,0,2] = sxx,sxy,sx
        matrix[...,1,0],matrix[...,1,1],matrix[...,1,2] = sxy,syy,sy
        matrix[...,2,0],matrix[...,2,1],matrix[...,2,2] = sx,sy,n
        rhs = np.stack([sxz,syz,sz],axis=-1)
        good = (n >= cfg.fit_min_cells) & (np.linalg.det(matrix) > 1e-8)
        model = np.full((h,w,3),np.nan)
        model[good] = np.linalg.solve(matrix[good],rhs[good])
    residual = np.maximum(0, szz-np.sum(model*rhs,axis=-1))/np.maximum(n,1)
    roughness = np.sqrt(residual)
    slope = np.degrees(np.arctan(np.hypot(model[...,0],model[...,1])))
    return model, slope, roughness, n


class TerrainGrid:
    def __init__(self, config=None):
        self.cfg = config or TerrainConfig()
        self.cfg.validate()
        self.voxels = {}
        self.seeds = {}
        self.scans = 0
        self.clipped_points = 0
        self.capacity_drops = 0
        self.snapshot = None
        self.preprocessing = {}
        self.wall_hits = {}

    def integrate_walls(self, points, imu_position):
        """Accumulate a separately selected wall band; no terrain relabeling."""
        p = np.asarray(points,dtype=float)
        if p.ndim != 2 or p.shape[1] != 3:
            raise ValueError('Expected Nx3 wall points')
        valid = (np.isfinite(p).all(axis=1)
                 & (np.linalg.norm(p-np.asarray(imu_position),axis=1) <= self.cfg.sensor_range)
                 & (np.abs(p[:,:2]) < self.cfg.max_extent/2).all(axis=1))
        keys = np.unique(np.floor(p[valid,:2]/self.cfg.resolution).astype(np.int64),axis=0)
        for row in keys:
            key = tuple(row)
            if key in self.wall_hits or len(self.wall_hits) < self.cfg.max_voxels:
                self.wall_hits[key] = min(self.wall_hits.get(key,0)+1,20)

    def integrate(self, points, imu_position):
        cfg = self.cfg
        p = np.asarray(points,dtype=float)
        origin = np.asarray(imu_position,dtype=float)
        if p.ndim != 2 or p.shape[1] != 3 or origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError('Expected Nx3 points and a finite XYZ sensor pose')
        finite = np.isfinite(p).all(axis=1)
        p = p[finite]
        near = (np.linalg.norm(p-origin,axis=1) <= cfg.sensor_range)
        within = (np.abs(p[:,:2]) < cfg.max_extent/2).all(axis=1)
        self.clipped_points += int(np.count_nonzero(~within))
        p = p[near & within]
        self.scans += 1
        if len(p) == 0:
            return
        # Seed only an observed, dominant low surface close to the robot.
        below = origin[2]-p[:,2]
        seed_mask = ((np.linalg.norm(p[:,:2]-origin[:2],axis=1) <= cfg.seed_radius)
                     & (below >= cfg.seed_min_below_imu) & (below <= cfg.seed_max_below_imu))
        candidates = p[seed_mask]
        if len(candidates) >= 8:
            levels = np.floor(candidates[:,2]/0.06).astype(np.int64)
            bins,counts = np.unique(levels,return_counts=True)
            level = (bins[np.argmax(counts)]+0.5)*0.06
            selected = candidates[np.abs(candidates[:,2]-level) <= 0.06]
            cells = np.floor(selected[:,:2]/cfg.terrain_cell_size).astype(np.int64)
            for key in np.unique(cells,axis=0):
                self.seeds[tuple(key)] = float(level)
        scale = np.array([cfg.resolution,cfg.resolution,cfg.vertical_resolution])
        keys = np.unique(np.floor(p/scale).astype(np.int64),axis=0)
        for row in keys:
            key = tuple(row)
            old = self.voxels.get(key,0)
            if old:
                self.voxels[key] = min(old+1,20)
            elif len(self.voxels) < cfg.max_voxels:
                self.voxels[key] = 1
            else:
                self.capacity_drops += 1

    def render(self):
        cfg = self.cfg
        rows = [(x,y,z,n) for (x,y,z),n in self.voxels.items() if n >= cfg.min_observations]
        if not rows:
            return None
        values = np.asarray(rows,dtype=float)
        fine = values[:,:2].astype(np.int64)
        z = (values[:,2]+0.5)*cfg.vertical_resolution
        factor = round(cfg.terrain_cell_size/cfg.resolution)
        coarse = np.floor_divide(fine,factor)
        cmin,cmax = coarse.min(axis=0)-cfg.fit_radius,coarse.max(axis=0)+cfg.fit_radius
        shape = tuple((cmax-cmin+1)[::-1])
        surface = np.full(shape,np.nan)
        groups, inverse = np.unique(coarse,axis=0,return_inverse=True)
        order = np.argsort(inverse,kind='stable')
        edges = np.r_[0,np.flatnonzero(np.diff(inverse[order]))+1,len(order)]
        for index, key in enumerate(groups):
            sample = z[order[edges[index]:edges[index+1]]]
            # Robust lower envelope: repeated floor returns under clutter.
            surface[key[1]-cmin[1],key[0]-cmin[0]] = np.quantile(sample,0.1)
        accepted = np.zeros(shape,dtype=bool)
        queue = deque()
        for key,level in self.seeds.items():
            x,y = np.asarray(key)-cmin
            if 0 <= y < shape[0] and 0 <= x < shape[1] and abs(surface[y,x]-level) <= 0.10:
                if not accepted[y,x]:
                    accepted[y,x] = True
                    queue.append((y,x))
        gradient = math.tan(math.radians(cfg.ground_connect_slope_deg))
        while queue:
            y,x = queue.popleft()
            for dy,dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                yy,xx = y+dy,x+dx
                if not (0 <= yy < shape[0] and 0 <= xx < shape[1]) or accepted[yy,xx]:
                    continue
                dz = abs(surface[yy,xx]-surface[y,x])
                if dz <= gradient*cfg.terrain_cell_size*math.hypot(dx,dy)+0.03:
                    accepted[yy,xx] = True
                    queue.append((yy,xx))
        ground = np.where(accepted,surface,np.nan)
        model,slope,roughness,support = fit_ground(ground,cfg)
        # A measured discontinuity is different from unobserved ground.
        step = np.zeros(shape,dtype=bool)
        for dy,dx in ((-1,0),(1,0),(0,-1),(0,1)):
            step |= np.abs(ground-shifted(ground,dy,dx)) > cfg.step_height
        wall_cells = np.asarray([key for key,n in self.wall_hits.items() if n >= cfg.min_observations],dtype=np.int64).reshape(-1,2)
        bounds = np.vstack([fine,wall_cells])
        lower,upper = bounds.min(axis=0)-1,bounds.max(axis=0)+1
        h,w = (upper-lower+1)[::-1]
        occupancy = np.full((h,w),-1,dtype=np.int8)
        costs = np.full((h,w),-1,dtype=np.int8)
        elevation = np.full((h,w),np.nan,dtype=np.float32)
        slopes = np.full((h,w),np.nan,dtype=np.float32)
        rough = np.full((h,w),np.nan,dtype=np.float32)
        confidence = np.zeros((h,w),dtype=np.float32)
        ci = coarse-cmin
        cy,cx = ci[:,1],ci[:,0]
        fit = model[cy,cx]
        xy = (fine+0.5)*cfg.resolution
        center = (coarse+0.5)*cfg.terrain_cell_size
        predicted = fit[:,2]+fit[:,0]*(xy[:,0]-center[:,0])+fit[:,1]*(xy[:,1]-center[:,1])
        relative = z-predicted
        valid = np.isfinite(predicted)
        floor = valid & (np.abs(relative) <= cfg.ground_tolerance)
        obstacle = valid & (relative >= cfg.obstacle_min_height) & (relative <= cfg.obstacle_max_height)
        drop = valid & (relative < -cfg.step_height)
        fy,fx = fine[:,1]-lower[1],fine[:,0]-lower[0]
        observed_ground = np.zeros((h,w),dtype=bool)
        observed_ground[fy[floor],fx[floor]] = True
        occupancy[fy[floor],fx[floor]] = 0
        occupancy[fy[obstacle],fx[obstacle]] = 100
        # Keep inferred terrain layers unknown wherever no ground was observed.
        s,r = slope[cy,cx],roughness[cy,cx]
        sc = np.clip((s-cfg.slope_cost_start_deg)/(cfg.slope_block_deg-cfg.slope_cost_start_deg),0,1)
        rc = np.clip((r-cfg.roughness_cost_start)/(cfg.roughness_block-cfg.roughness_cost_start),0,1)
        severity = np.maximum(sc,rc)
        terrain = np.rint(80*severity)
        hazard = step[cy,cx] | (s >= cfg.slope_block_deg) | (r >= cfg.roughness_block) | drop
        terrain[hazard] = 100
        costs[fy[floor],fx[floor]] = terrain[floor].astype(np.int8)
        costs[fy[drop],fx[drop]] = 100
        costs[fy[obstacle],fx[obstacle]] = 100
        for array,source in ((elevation,predicted),(slopes,s),(rough,r)):
            array[fy[floor],fx[floor]] = source[floor]
        confidence[fy[floor],fx[floor]] = np.minimum(values[floor,3]/5,1)*np.minimum(support[cy[floor],cx[floor]]/12,1)
        combined = occupancy.copy()
        observed = costs >= 0
        combined[observed] = costs[observed]
        combined[occupancy == 100] = 100
        walls = np.full((h,w),-1,dtype=np.int8)
        walls[observed_ground] = 0
        if len(wall_cells):
            walls[wall_cells[:,1]-lower[1],wall_cells[:,0]-lower[0]] = 100
        result = dict(occupancy=occupancy,cost=costs,traversability=combined,walls=walls,
                      elevation=elevation,slope_deg=slopes,roughness=rough,confidence=confidence,
                      origin=lower*cfg.resolution,resolution=cfg.resolution)
        self.snapshot = result
        return result

    def save(self, stem, frame='odom'):
        data = self.render()
        if data is None or not np.any(data['traversability'] >= 0):
            raise ValueError('No classified map to save yet')
        stem = Path(stem).expanduser().resolve()
        stem.parent.mkdir(parents=True,exist_ok=True)
        for key,suffix in [('occupancy',''),('traversability','_traversability'),('walls','_walls')]:
            target = stem.with_name(stem.name+suffix)
            image = np.full(data[key].shape,205,dtype=np.uint8)
            image[data[key] == 0] = 254
            image[data[key] >= 65] = 0
            moderate = (data[key] > 0) & (data[key] < 65)
            image[moderate] = np.rint(254*(1-data[key][moderate]/100)).astype(np.uint8)
            pgm = target.with_suffix('.pgm')
            with pgm.open('wb') as stream:
                stream.write(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode())
                stream.write(np.flipud(image).tobytes())
            # JSON strings are valid YAML strings, including paths with spaces.
            text = (f'image: {json.dumps(pgm.name)}\nmode: trinary\nresolution: {cfg_float(data["resolution"])}\n'
                    f'origin: [{data["origin"][0]}, {data["origin"][1]}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
            target.with_suffix('.yaml').write_text(text)
        np.savez_compressed(stem.with_suffix('.npz'),**data,frame=frame,config=json.dumps(vars(self.cfg)),
                            preprocessing=json.dumps(self.preprocessing))
        stem.with_suffix('.json').write_text(json.dumps(dict(frame=frame,scans=self.scans,
            voxels=len(self.voxels),clipped_points=self.clipped_points,capacity_drops=self.capacity_drops,
            provisional_thresholds=True,config=vars(self.cfg),preprocessing=self.preprocessing),indent=2)+'\n')
        return str(stem.with_suffix('.yaml'))


def cfg_float(value):
    return format(value,'.12g')
