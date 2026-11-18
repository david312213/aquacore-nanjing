"""Short-term obstacle memory for EGO's planning cloud (numpy only, no ROS).

EGO rebuilds its local map from every single cloud, and FAST-LIO drops points
inside its blind radius, so a door frame vanishes from the map as the aircraft
reaches it. This keeps points already seen, per voxel, around the aircraft.
Historical voxels are retained once seen in min_hits different scans. The
current scan is also published immediately, including unconfirmed returns.
"""
import numpy as np

_OFFSET = 1 << 20  # index offset per axis; ±1 048 576 voxels


class VoxelMemory:
    def __init__(self, voxel_m=.10, radius_m=4., max_age_s=60., min_hits=2, z_max=None):
        self.voxel, self.radius, self.max_age = float(voxel_m), float(radius_m), float(max_age_s)
        self.min_hits, self.z_max = int(min_hits), z_max
        self.keys = np.zeros(0, dtype=np.int64)
        self.sums = np.zeros((0, 3))
        self.counts = np.zeros(0, dtype=np.int64)
        self.hits = np.zeros(0, dtype=np.int64)
        self.last_seen = np.zeros(0)

    def __len__(self):
        return int(self.keys.size)

    def _keys(self, points):
        index = np.floor(points/self.voxel).astype(np.int64)+_OFFSET
        return (index[:, 0] << 42) | (index[:, 1] << 21) | index[:, 2]

    def crop(self, points, center_xy):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        keep = np.hypot(points[:, 0]-center_xy[0], points[:, 1]-center_xy[1]) <= self.radius
        if self.z_max is not None:
            keep &= points[:, 2] <= self.z_max
        return points[keep]

    def add(self, points, stamp, center_xy):
        """Merge one scan; points are already cropped. Returns the published cloud."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        # Expired evidence must not count as confirmation of a new observation.
        self.prune(stamp, center_xy)
        if points.size:
            keys, inverse = np.unique(self._keys(points), return_inverse=True)
            sums = np.zeros((keys.size, 3))
            np.add.at(sums, inverse, points)
            counts = np.bincount(inverse, minlength=keys.size)
            position = np.searchsorted(self.keys, keys)
            found = position < self.keys.size
            found[found] = self.keys[position[found]] == keys[found]
            old = position[found]
            self.sums[old] += sums[found]
            self.counts[old] += counts[found]
            self.hits[old] += 1  # one hit per scan, however many points
            self.last_seen[old] = stamp
            new = ~found
            if np.any(new):
                order = np.argsort(np.concatenate((self.keys, keys[new])), kind='stable')
                self.keys = np.concatenate((self.keys, keys[new]))[order]
                self.sums = np.concatenate((self.sums, sums[new]))[order]
                self.counts = np.concatenate((self.counts, counts[new]))[order]
                self.hits = np.concatenate((self.hits, np.ones(np.count_nonzero(new), dtype=np.int64)))[order]
                self.last_seen = np.concatenate((self.last_seen, np.full(np.count_nonzero(new), float(stamp))))[order]
        self.prune(stamp, center_xy)
        return self.cloud(points)

    def prune(self, stamp, center_xy):
        """Expire cached obstacles without adding scans or refreshing evidence."""
        if not self.keys.size:
            return
        means = self.sums/self.counts[:, None]
        keep = ((stamp-self.last_seen <= self.max_age) &
                (np.hypot(means[:, 0]-center_xy[0], means[:, 1]-center_xy[1]) <= self.radius))
        if not np.all(keep):
            self.keys, self.sums, self.counts = self.keys[keep], self.sums[keep], self.counts[keep]
            self.hits, self.last_seen = self.hits[keep], self.last_seen[keep]

    def cloud(self, current=None):
        """Remembered voxel means plus the current scan (never less than before)."""
        confirmed = self.hits >= self.min_hits
        remembered = self.sums[confirmed]/self.counts[confirmed][:, None]
        if current is None or not len(current):
            return remembered
        return np.vstack((remembered, np.asarray(current, dtype=float).reshape(-1, 3)))


def pointcloud2_fields(field_type):
    return [field_type(name=name, offset=4*index, datatype=7, count=1) for index, name in enumerate('xyz')]


def xyz_bytes(points):
    return np.ascontiguousarray(np.asarray(points, dtype='<f4').reshape(-1, 3)).tobytes()
