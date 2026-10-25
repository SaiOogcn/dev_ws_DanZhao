"""Explicit SE(2) alignment of an external body/sensor pose to the field.

No first-message auto-alignment: a wrong start position must remain detectable.
"""
import math


def field_body_pose(pose,field_offset=(0.,0.,0.),body_to_sensor=(0.,0.,0.)):
    if not all(math.isfinite(v) for v in (*pose,*field_offset,*body_to_sensor)):
        raise ValueError('nonfinite localization pose or extrinsics')
    x,y,yaw=pose;ox,oy,angle=field_offset;sx,sy,sa=body_to_sensor
    sensor_yaw=yaw+angle
    body_yaw=sensor_yaw-sa
    c,s=math.cos(angle),math.sin(angle)
    bx=ox+c*x-s*y-math.cos(body_yaw)*sx+math.sin(body_yaw)*sy
    by=oy+s*x+c*y-math.sin(body_yaw)*sx-math.cos(body_yaw)*sy
    return bx,by,math.atan2(math.sin(body_yaw),math.cos(body_yaw))


def interpolate_pose(history,stamp,max_gap=.20):
    """Interpolate laser-time poses; do not smear a scan using today's pose."""
    rows=list(history)
    for i in range(len(rows)-1,-1,-1):
        t,a=rows[i]
        if t<=stamp:
            if i+1<len(rows):
                u,b=rows[i+1]
                if u-t>max_gap:return None
                ratio=(stamp-t)/max(u-t,1e-12)
                dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
                return (a[0]+ratio*(b[0]-a[0]),a[1]+ratio*(b[1]-a[1]),a[2]+ratio*dyaw)
            return a if abs(stamp-t)<1e-9 else None
    return None
