"""ROS-free diagnostic scoring/rasterisation. Never interpolates reachability."""
import math
import numpy as np
from .core import angle
from .volume import sector_penalty

COMPONENTS = ('travel', 'destination', 'distance', 'facing', 'total')


def components(pose, arm, bearing, target, travel, destination, cfg):
    distance = math.hypot(pose[0]-target[0], pose[1]-target[1])
    values = {
        'travel': cfg['weight_navigation']*travel/cfg['length_scale'],
        'destination': cfg['weight_destination_cost']*destination,
        'distance': cfg['weight_working_distance']*((distance-cfg[arm+'_preferred_distance'])/cfg['distance_scale'])**2,
        'facing': cfg['weight_facing']*sector_penalty(bearing, cfg[arm+'_facing_min_deg'], cfg[arm+'_facing_max_deg']),
    }
    values['total'] = sum(values.values())
    if not all(math.isfinite(v) and v >= 0 for v in values.values()):
        raise ValueError('Score components must be finite and non-negative')
    return values


def heading_matches(yaw, enabled, degrees, tolerance):
    return not enabled or abs(angle(yaw-math.radians(degrees))) <= math.radians(tolerance)+1e-12


class ScoreRaster:
    """Each cell shows one real candidate: the minimum TOTAL cost in that cell.

    All component layers use that same winner; component-wise independent minima
    would describe an impossible mixture of headings/arms.
    """
    def __init__(self, rows, resolution, max_cells):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Grid resolution must be positive')
        self.resolution = resolution
        if rows:
            xy = np.asarray([r[0][:2] for r in rows], dtype=float)
            if not np.isfinite(xy).all(): raise ValueError('Nonfinite candidate position')
            lo = np.floor(xy.min(axis=0)/resolution+1e-10)
            hi = np.floor(xy.max(axis=0)/resolution+1e-10)
        else:
            lo = hi = np.zeros(2)
        self.lo = lo.astype(int)
        self.origin = tuple(lo*resolution)
        self.width, self.height = map(int, hi-lo+1)
        if self.width*self.height > max_cells:
            raise ValueError('Score grid exceeds max_cells; increase grid_resolution')

    def index(self, pose):
        x, y = np.floor(np.asarray(pose[:2])/self.resolution+1e-10).astype(int)-self.lo
        if not (0 <= x < self.width and 0 <= y < self.height): raise ValueError('Candidate outside score grid')
        return int(y*self.width+x)

    def render(self, records, scale):
        if not math.isfinite(scale) or scale <= 0: raise ValueError('score_max must be positive')
        size = self.width*self.height
        winners = {}
        states = {}
        for record in records:
            cell = self.index(record['pose'])
            states.setdefault(cell, set()).add(record['state'])
            values = record.get('scores')
            if values is not None and (cell not in winners or values['total'] < winners[cell]['scores']['total']):
                winners[cell] = record
        layers = {key: np.full(size, -1, dtype=np.int8) for key in COMPONENTS}
        state = np.full(size, -1, dtype=np.int8)
        for cell, kinds in states.items():
            # 0: at least one scored candidate; 50: unknown/unchecked remains;
            # 100: every candidate in this cell was rejected by detailed checks.
            state[cell] = 0 if cell in winners else (50 if 'unchecked' in kinds or 'no_estimate' in kinds else 100)
        for cell, record in winners.items():
            for key in COMPONENTS:
                layers[key][cell] = int(np.clip(np.rint(100*record['scores'][key]/scale), 0, 100))
        layers['state'] = state
        ranges = {key: {'min': min((r['scores'][key] for r in winners.values()), default=None),
                        'max': max((r['scores'][key] for r in winners.values()), default=None),
                        'clipped_cells': sum(r['scores'][key] > scale for r in winners.values())}
                  for key in COMPONENTS}
        return layers, winners, ranges
