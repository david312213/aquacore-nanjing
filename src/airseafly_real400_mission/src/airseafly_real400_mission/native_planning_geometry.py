"""One frozen FAST-LIO world transform for planning and target observations.

Environment points already include the sensor extrinsics that registered them
in FAST-LIO world. Only the fixed world transform applies to them: the vehicle
IMU lever arm belongs exclusively to vehicle odometry, never to map targets.
This module deliberately has no ROS import so its boundary can be replayed.
"""
from copy import deepcopy

import numpy as np

from .native_motion import quaternion, rotation, vector
from .native_reference import LIO_FRAME


RAW_LIO_FRAMES = frozenset(('world', 'camera_init'))


class RawToLocal:
    def __init__(self, quaternion_xyzw, translation_xyz, source_frame='world'):
        self.rotation = rotation(quaternion_xyzw)
        self.translation = vector(translation_xyz).copy()
        self.source_frame = str(source_frame)
        self.rotation.setflags(write=False)
        self.translation.setflags(write=False)

    @classmethod
    def from_message(cls, message):
        if message.header.frame_id != LIO_FRAME:
            raise ValueError('raw transform must have the corrected radar parent frame')
        q, t = message.transform.rotation, message.transform.translation
        return cls((q.x, q.y, q.z, q.w), (t.x, t.y, t.z), message.child_frame_id)

    @classmethod
    def from_frozen_frame(cls, frame, source_frame='world'):
        if frame.r_nw is None:
            raise ValueError('FAST-LIO world transform is not initialized')
        return cls(quaternion(frame.r_nw), frame.t_nw, source_frame)

    def accepts(self, frame_id):
        return (frame_id == self.source_frame or
                self.source_frame in RAW_LIO_FRAMES and frame_id in RAW_LIO_FRAMES)

    def point(self, xyz):
        return tuple(self.rotation @ vector(xyz) + self.translation)

    def points(self, xyz):
        points = np.asarray(xyz, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('expected finite environment points with shape (N, 3)')
        return points @ self.rotation.T + self.translation


def transform_observation_packet(packet, transform):
    """Copy a front-camera packet and convert its environment points once.

The front worker keeps its image-time raw-IMU/camera calibration. Frame tagging
at this boundary prevents an already converted packet from being offset twice.
Unprojected detections, image timestamps and source records remain untouched.
"""
    out = deepcopy(packet)
    source = out.get('frame_id', 'world')
    if source == LIO_FRAME:
        return out
    if not transform.accepts(source):
        raise ValueError('unsupported front observation frame: '+str(source))
    for observation in out.get('observations', []):
        raw = observation['position_xyz']
        observation['raw_position_xyz'] = list(raw)
        observation['position_xyz'] = list(transform.point(raw))
    out['raw_frame_id'] = source
    out['frame_id'] = LIO_FRAME
    return out


def planning_odometry(body_odometry):
    """EGO contract: body-origin pose, WORLD-frame linear/angular twist.

MAVROS retains the original REP-147 body-twist message. EGO directly reads
twist.linear as world velocity, so its separate topic intentionally has this
legacy planner convention while retaining the body pose/child reference.
"""
    out = deepcopy(body_odometry)
    q = out.pose.pose.orientation
    r = rotation((q.x, q.y, q.z, q.w))
    out.header.frame_id = LIO_FRAME
    for field in ('linear', 'angular'):
        v = getattr(out.twist.twist, field)
        v.x, v.y, v.z = r @ vector((v.x, v.y, v.z))
    if hasattr(out.twist, 'covariance'):
        c = np.asarray(out.twist.covariance, dtype=float).reshape(6, 6)
        basis = np.zeros((6, 6))
        basis[:3, :3] = basis[3:, 3:] = r
        out.twist.covariance = list((basis @ c @ basis.T).reshape(-1))
    return out


def transform_registered_cloud(message, transform):
    """Transform XYZ only, preserving layout, ancillary fields and scan stamp.

Works on a PointCloud2-shaped object without importing ROS. Row/point padding,
organized dimensions, endianness, and FLOAT32/FLOAT64 XYZ are preserved. Invalid
points remain byte-for-byte unchanged and never become artificial obstacles.
"""
    out = deepcopy(message)
    if out.header.frame_id == LIO_FRAME:
        return out
    if not transform.accepts(out.header.frame_id):
        raise ValueError('unsupported registered cloud frame: '+str(out.header.frame_id))
    height, width = int(out.height), int(out.width)
    point_step, row_step = int(out.point_step), int(out.row_step)
    if (height < 0 or width < 0 or point_step <= 0 or
            row_step < width*point_step or len(out.data) < height*row_step):
        raise ValueError('invalid PointCloud2 dimensions or data length')
    fields = {}
    for field in out.fields:
        if field.name in ('x', 'y', 'z'):
            if field.name in fields:
                raise ValueError('duplicate PointCloud2 coordinate field')
            fields[field.name] = field
    if set(fields) != {'x', 'y', 'z'}:
        raise ValueError('PointCloud2 requires x, y and z fields')
    buffer = bytearray(out.data)
    columns = []
    spans = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('PointCloud2 coordinates must be scalar FLOAT32/FLOAT64')
        dtype = np.dtype(('>' if out.is_bigendian else '<') + ('f4' if field.datatype == 7 else 'f8'))
        offset, end = int(field.offset), int(field.offset)+dtype.itemsize
        if offset < 0 or end > point_step or any(offset < b and end > a for a, b in spans):
            raise ValueError('invalid or overlapping PointCloud2 coordinate offsets')
        spans.append((offset, end))
        if height and width:
            columns.append(np.ndarray((height, width), dtype=dtype, buffer=buffer,
                                      offset=offset, strides=(row_step, point_step)))
    if height and width:
        xyz = np.stack(columns, axis=-1).astype(float)
        finite = np.isfinite(xyz).all(axis=-1)
        transformed = transform.points(xyz[finite])
        for axis, column in enumerate(columns):
            column[finite] = transformed[:, axis]
    out.data = bytes(buffer)
    out.header.frame_id = LIO_FRAME
    return out
