"""Full local A/B/C navigation course, using the supplied field geometry.

QR and captioning are intentionally outside this navigation test. The direction
is an explicit scenario setting, and is never presented as a decoded QR result.
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .course_layout import OfficialLayout, OUTER, INNER

START = (.54, .30, 0.)
FRONT, REAR, HALF_WIDTH = .2015, .1375, .1095
LAYOUT = OfficialLayout()


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def resample(points, spacing=.035):
    out = [tuple(points[0])]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.dist(a,b)/spacing))
        out.extend((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n) for i in range(1,n+1))
    return out


def line(a,b):
    return resample([a,b])


def arc(cx,cy,r,a,b):
    n=max(4,math.ceil(abs(b-a)*r/.025))
    return [(cx+r*math.cos(a+(b-a)*i/n),cy+r*math.sin(a+(b-a)*i/n)) for i in range(n+1)]


def bezier(a,b,c,d,n=140):
    return [tuple((1-t)**3*a[k]+3*(1-t)**2*t*b[k]+3*(1-t)*t*t*c[k]+t**3*d[k] for k in (0,1))
            for t in (i/(n-1) for i in range(n))]


def cat(*parts):
    out=[]
    for part in parts:
        for p in part:
            if not out or math.dist(out[-1],p)>1e-7:
                out.append(tuple(p))
    return out


def car_boundary(pose, margin=0.):
    x,y,yaw=pose
    c,s=math.cos(yaw),math.sin(yaw)
    front,rear,half=FRONT+margin,REAR+margin,HALF_WIDTH+margin
    uv=[]
    for i in range(7):
        u=-rear+(front+rear)*i/6
        uv.extend([(u,-half),(u,half)])
    for i in range(7):
        v=-half+2*half*i/6
        uv.extend([(-rear,v),(front,v)])
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in uv]


def legal_pose(pose, margin=.012):
    return all(allowed_point(x,y) for x,y in car_boundary(pose,margin))


def rounded_distance(x,y,b):
    x0,y0,x1,y1,r=b
    dx=abs(x-(x0+x1)/2)-(x1-x0)/2+r
    dy=abs(y-(y0+y1)/2)-(y1-y0)/2+r
    return math.hypot(max(dx,0),max(dy,0))+min(max(dx,dy),0)-r


def allowed_point(x,y):
    if not (0<=x<=5 and 0<=y<=5):return False
    if y<2:return True
    if y<2.5:return 2<=x<=3
    ring=rounded_distance(x,y,OUTER)<=0 and rounded_distance(x,y,INNER)>=0
    return ring or (2<=x<=3 and y<=OUTER[1])


def path_yaw(points,i,direction=1):
    a=points[max(0,i-1)];b=points[min(len(points)-1,i+1)]
    return wrap(math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if direction<0 else 0.))


@dataclass
class Stage:
    name: str
    points: list
    direction: int = 1
    max_speed: float = .25
    replan: bool = False


def _ring_ccw():
    # Centerline fitted inside both actual printed annulus boundaries.
    xl,xr,yb,yt,r=.75,4.25,3.17,4.31,.40
    return cat(line((xl+r,yb),(xr-r,yb)),arc(xr-r,yb+r,r,-math.pi/2,0),
        line((xr,yb+r),(xr,yt-r)),arc(xr-r,yt-r,r,0,math.pi/2),
        line((xr-r,yt),(xl+r,yt)),arc(xl+r,yt-r,r,math.pi/2,math.pi),
        line((xl,yt-r),(xl,yb+r)),arc(xl+r,yb+r,r,math.pi,3*math.pi/2),
        line((xl+r,yb),(xr-r,yb)))


def ring_route(direction):
    # Entry and exit use the two sides of the same 1m B-channel opening.
    # Traversal visits all four corners once, then exits at the junction.
    base=_ring_ccw()
    # Remove duplicate bottom straight from the cyclic sequence.
    end_index=next(i for i,p in enumerate(base[1:],1) if i>len(base)//2 and math.dist(p,base[0])<1e-6)
    base=base[:end_index]
    if direction=='CW':base=list(reversed(base))
    start=(2.32,3.17) if direction=='CW' else (3.08,3.17)
    end=(2.88,3.17) if direction=='CW' else (2.12,3.17)
    idx=min(range(len(base)),key=lambda i:math.dist(base[i],start))
    ordered=base[idx:]+base[:idx]+base[idx:idx+1]
    travelled=0.;result=[start]
    for p in ordered:
        travelled+=math.dist(result[-1],p)
        result.append(p)
        if travelled>4 and math.dist(p,end)<.04:
            result[-1]=end
            return resample(result)
    raise ValueError('Could not join ring exit')


def build_course(package_share, direction='CW'):
    if direction not in ('CW','CCW'):raise ValueError('direction must be CW or CCW')
    source=json.loads((Path(package_share)/'config/full_race/parking_entry.json').read_text())
    rows=source['rows']
    xy=[(r[0],r[1]) for r in rows]
    heading=math.radians(30.2)
    goal=(4.2,1.5)
    outbound=bezier(START[:2],(1.85,.30),(goal[0]-.95*math.cos(heading),goal[1]-.95*math.sin(heading)),goal)
    stages=[Stage('OUTBOUND_A',outbound,1,1.25,True),
            Stage('TASK_AREA_APPROACH',resample(xy[:4]),1,.30),
            Stage('REVERSE_TRANSFER_A',resample([xy[3]]+xy[4:30]),-1,.60,True),
            Stage('ENTER_B_CHANNEL',resample([xy[29]]+xy[30:72]),1,1.0,True)]
    if direction=='CW':
        entry=arc(2.32,2.79,.38,0,math.pi/2)
        exit_=arc(2.88,2.79,.38,math.pi/2,math.pi)
    else:
        entry=arc(3.08,2.79,.38,math.pi,math.pi/2)
        exit_=arc(2.12,2.79,.38,math.pi/2,0)
    stages.extend([Stage('ENTER_C_RING',entry,1,.55,True),
                   Stage('C_RING_'+direction,ring_route(direction),1,1.20,True),
                   Stage('EXIT_C_RING',exit_,1,.55,True),
                   Stage('EXIT_B_CHANNEL',line((2.5,2.79),(2.5,1.73)),1,1.0,True)])
    returning=bezier((2.5,1.73),(2.5,1.10),(1.05,.30),START[:2])
    stages.append(Stage('RETURN_A_AND_PARK',returning,1,1.15,True))
    return stages


def audit_course(stages):
    bad=[];jumps=[];length=0.
    for j,stage in enumerate(stages):
        if j and math.dist(stages[j-1].points[-1],stage.points[0])>.015:
            jumps.append((stage.name,math.dist(stages[j-1].points[-1],stage.points[0])))
        length+=sum(math.dist(a,b) for a,b in zip(stage.points,stage.points[1:]))
        for i,p in enumerate(stage.points):
            if not legal_pose((*p,path_yaw(stage.points,i,stage.direction))):
                bad.append((stage.name,i,p))
    return {'length_m':length,'illegal_reference_poses':bad,'stage_gaps':jumps}
