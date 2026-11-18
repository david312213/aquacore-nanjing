"""PointCloud2 XYZ reading, independent of retired door detection."""
import numpy as np

def cloud_xyz(message):
    """Finite XYZ rows of a PointCloud2-shaped object, as float64 Nx3."""
    height, width = int(message.height), int(message.width)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if height <= 0 or width <= 0:
        return np.zeros((0, 3))
    if point_step <= 0 or row_step < width*point_step or len(message.data) < height*row_step:
        raise ValueError('invalid PointCloud2 dimensions or data length')
    fields = {f.name: f for f in message.fields if f.name in ('x', 'y', 'z')}
    if set(fields) != {'x', 'y', 'z'}:
        raise ValueError('PointCloud2 requires x, y and z fields')
    buffer = bytes(message.data)
    columns = []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('PointCloud2 coordinates must be scalar FLOAT32/FLOAT64')
        dtype = np.dtype(('>' if message.is_bigendian else '<') + ('f4' if field.datatype == 7 else 'f8'))
        if int(field.offset)+dtype.itemsize > point_step:
            raise ValueError('invalid PointCloud2 coordinate offset')
        columns.append(np.ndarray((height, width), dtype=dtype, buffer=buffer,
                                  offset=int(field.offset), strides=(row_step, point_step)))
    xyz = np.stack(columns, axis=-1).reshape(-1, 3).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]

