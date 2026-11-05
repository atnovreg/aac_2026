"""Preserve original point records while partitioning a sensor-frame exclusion ROI."""
import copy
import numpy as np

DTYPE=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
    'formats':['<f4']*4+['u1','u1','<f8'], 'offsets':[0,4,8,12,16,17,18], 'itemsize':26})


def validate_boxes(boxes):
    if not isinstance(boxes,list) or not boxes: raise ValueError('At least one exclusion box is required')
    for box in boxes:
        lo=np.asarray(box['min'],dtype=float);hi=np.asarray(box['max'],dtype=float)
        if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all() or (lo>=hi).any():
            raise ValueError('Exclusion box requires finite min < max for x,y,z')
    return boxes


def records(msg):
    expected=list(zip(DTYPE.names,[0,4,8,12,16,17,18],[7,7,7,7,2,2,8]))
    fields={field.name:(field.offset,field.datatype,field.count) for field in msg.fields}
    if (msg.is_bigendian or msg.point_step!=26 or len(fields)!=7 or
        any(fields.get(name)!=(offset,kind,1) for name,offset,kind in expected) or
        msg.row_step<msg.width*26 or len(msg.data)!=msg.row_step*msg.height):
        raise ValueError('Expected recorded little-endian Livox 26-byte PointCloud2 layout')
    return np.ndarray((msg.height,msg.width),dtype=DTYPE,buffer=bytes(msg.data),
                      strides=(msg.row_step,26)).reshape(-1)


def exclusion_mask(points, boxes):
    validate_boxes(boxes)
    xyz=np.c_[points['x'],points['y'],points['z']]
    removed=np.zeros(len(points),dtype=bool)
    for box in boxes:
        removed |= ((xyz>=box['min']) & (xyz<=box['max'])).all(axis=1)
    return removed


def filter_message(msg, boxes):
    points=records(msg)
    removed=exclusion_mask(points,boxes)
    return partition_message(msg, removed)


def partition_message(msg, removed):
    points=records(msg)
    removed=np.asarray(removed,dtype=bool)
    if removed.shape!=(len(points),): raise ValueError("Point selection size mismatch")
    outputs=[]
    for selection in (~removed,removed):
        out=copy.copy(msg)
        out.height=1;out.width=int(selection.sum());out.row_step=out.width*26
        out.data=points[selection].tobytes()
        outputs.append(out)
    return tuple(outputs)


def mapper_effective_timestamps(points, point_filter_num=3, blind=.5):
    """Mirror the recorded feature-disabled Livox input policy for timing audits.

    Adapter finite filtering and stable sorting precede FAST-LIO's skip-first,
    valid-tag/line counter, every-N selection and spherical blind test.
    """
    finite=np.isfinite(np.c_[points['x'],points['y'],points['z'],points['intensity']]).all(axis=1)
    points=points[finite]
    points=points[np.argsort(points['timestamp'],kind='stable')][1:]
    valid=(points['line']<4)&np.isin(points['tag']&0x30,[0,0x10])
    selected=points[valid][point_filter_num-1::point_filter_num]
    ranges2=selected['x']**2+selected['y']**2+selected['z']**2
    return selected['timestamp'][ranges2>blind**2]
