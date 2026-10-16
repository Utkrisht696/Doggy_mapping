"""PointCloud2 validation/filtering with unchanged point records and coordinates."""
import numpy as np


def cloud_arrays(msg):
    fields={field.name:field for field in msg.fields}
    if any(name not in fields or fields[name].datatype!=7 or fields[name].count!=1 for name in ('x','y','z')):
        raise ValueError('Expected scalar float32 x/y/z fields')
    if msg.point_step<=0 or msg.row_step<msg.width*msg.point_step:
        raise ValueError('Invalid PointCloud2 record/row size')
    if len(msg.data)<msg.row_step*msg.height:
        raise ValueError('Truncated PointCloud2 data')
    dtype=np.dtype({'names':['x','y','z'],
                    'formats':[('>' if msg.is_bigendian else '<')+'f4']*3,
                    'offsets':[fields[name].offset for name in ('x','y','z')],
                    'itemsize':msg.point_step})
    values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                      strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([values[name] for name in ('x','y','z')])
    records=np.ndarray((msg.height,msg.width,msg.point_step),dtype=np.uint8,buffer=msg.data,
                       strides=(msg.row_step,msg.point_step,1)).reshape(-1,msg.point_step)
    valid=np.isfinite(xyz).all(axis=1)&~np.equal(xyz,0).all(axis=1)
    return xyz,records,valid
