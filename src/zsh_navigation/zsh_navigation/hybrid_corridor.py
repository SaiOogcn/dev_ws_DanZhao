"""Forward/reverse heading lattice for a whole observed A-zone cone corridor.

Primitives obey the identified Ackermann curvature bound. No cone truth is
used: input consists only of accumulated scan endpoints. Every primitive and
terminal connector is checked with the full oriented body and base inflation.
"""
from heapq import heappush,heappop
import itertools
import math
import numpy as np
import time
from .race_mpc import SamplingMPC


class HybridCorridor:
    def __init__(self,extra_boundary=.015,boundary_margin=.035):
        self.geometry=SamplingMPC(boundary_margin=boundary_margin)
        # Leave tracking room around the nominal lattice, in addition to the
        # execution controller's independent 3.5cm boundary check.
        self.geometry.u+=np.sign(self.geometry.u)*extra_boundary
        self.geometry.w+=np.sign(self.geometry.w)*extra_boundary
        self.geometry.obstacle_margin+=extra_boundary
        self.expansions=0
        self.reason='UNPLANNED'
        self.time_budget=None
        self.analytic_connections=False
        self.analytic_approach=False
        self.curvature_limit=None

    @staticmethod
    def wrap(value):return math.atan2(math.sin(value),math.cos(value))

    def connector(self,pose,goal,heading,direction,obstacles,inflation,start_scale=1.,end_scale=1.):
        offset=self.geometry.axle_offset
        start=np.asarray(pose[:2])-offset*np.array([math.cos(pose[2]),math.sin(pose[2])])
        end=np.asarray(goal)-offset*np.array([math.cos(heading),math.sin(heading)])
        distance=np.linalg.norm(end-start)
        if distance<.03:
            return [tuple(goal)] if abs(self.wrap(pose[2]-heading))<.18 else None
        t=np.linspace(0.,1.,max(6,math.ceil(distance/.025)))[:,None]
        u=distance*start_scale*np.array([math.cos(pose[2]),math.sin(pose[2])])*direction
        v=distance*end_scale*np.array([math.cos(heading),math.sin(heading)])*direction
        p=(2*t**3-3*t**2+1)*start+(t**3-2*t**2+t)*u+(-2*t**3+3*t**2)*end+(t**3-t**2)*v
        d=(6*t*t-6*t)*start+(3*t*t-4*t+1)*u+(-6*t*t+6*t)*end+(3*t*t-2*t)*v
        dd=(12*t-6)*start+(6*t-4)*u+(-12*t+6)*end+(6*t-2)*v
        curvature=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.maximum(np.linalg.norm(d,axis=1)**3,1e-9)
        if np.max(curvature)>(self.curvature_limit or 2.65):return None
        yaw=np.arctan2(d[:,1],d[:,0])+(math.pi if direction<0 else 0.)
        p+=offset*np.c_[np.cos(yaw),np.sin(yaw)]
        if not np.all(self.geometry.clear(p[:,0],p[:,1],yaw,obstacles,inflation)):return None
        return [tuple(a) for a in p[1:]]

    def plan(self,pose,goal,goal_yaw,points,inflation,preferred,direction=1,local_corridor=False):
        obs=np.asarray(getattr(points,'boxes',points),dtype=float)
        if obs.size==0:obs=np.empty((0,2))
        if not self.geometry.clear(np.array([goal[0]]),np.array([goal[1]]),np.array([goal_yaw]),obs,inflation,recovery=False)[0]:
            self.reason='GOAL_FOOTPRINT_BLOCKED';self.expansions=0;return []
        reference=np.asarray(preferred)
        if self.analytic_connections:
            # Preserve a clear final tangent when a single cubic would sweep
            # its bumper through a cone beside the phase junction. Prefer the
            # straight reverse capture so the next forward channel starts with
            # aligned wheels and body heading.
            for lead in ((.30,.50,.70,.90) if self.analytic_approach else ()):
                approach=(goal[0]-direction*lead*math.cos(goal_yaw),
                          goal[1]-direction*lead*math.sin(goal_yaw))
                tail=self.connector((*approach,goal_yaw),goal,goal_yaw,direction,obs,inflation)
                if tail is None:continue
                for start_scale,end_scale in itertools.product((.5,.8,1.,1.3,1.6),repeat=2):
                    curve=self.connector(pose,approach,goal_yaw,direction,obs,inflation,start_scale,end_scale)
                    if curve:
                        self.reason='ANALYTIC_APPROACH_FOUND';self.expansions=0
                        return [tuple(pose[:2])]+curve+tail
            for start_scale,end_scale in itertools.product((.5,.8,1.,1.3,1.6),repeat=2):
                curve=self.connector(pose,goal,goal_yaw,direction,obs,inflation,start_scale,end_scale)
                if curve:
                    self.reason='ANALYTIC_CURVE_FOUND';self.expansions=0
                    return [tuple(pose[:2])]+curve
        gain=.70 if direction>0 else .80
        curvature=gain*np.tan(np.array([-.52,-.30,0.,.30,.52]))/.143
        if self.curvature_limit is not None:
            curvature=self.curvature_limit*np.array([-1.,-.55,0.,.55,1.])
        length=.16;ds=length/4
        def key(p):return (round(p[0]/.07),round(p[1]/.07),round((p[2]%(2*math.pi))/(2*math.pi)*48)%48)
        def heuristic(p):return math.hypot(p[0]-goal[0],p[1]-goal[1])
        serial=itertools.count();first=key(pose);states={first:(*pose,)}
        costs={first:0.};came={};segments={}
        queue=[(heuristic(pose),0.,next(serial),first)]
        self.expansions=0
        began=time.perf_counter()
        while queue and self.expansions<24000:
            if self.time_budget is not None and self.expansions%64==0 and time.perf_counter()-began>self.time_budget:
                self.reason='PLANNING_TIME_BUDGET';return []
            _,cost,_,node=heappop(queue)
            if cost>costs.get(node,math.inf)+1e-9:continue
            p=states[node];self.expansions+=1
            if heuristic(p)<.50:
                tail=self.connector(p,goal,goal_yaw,direction,obs,inflation)
                if tail is not None:
                    parts=[tail]
                    while node in came:parts.append(segments[node]);node=came[node]
                    route=[tuple(pose[:2])]
                    for part in reversed(parts):route.extend(part)
                    self.reason='HYBRID_PATH_FOUND';return route
            x=np.full(5,float(p[0]));y=np.full(5,float(p[1]));yaw=np.full(5,float(p[2]));safe=np.ones(5,dtype=bool)
            samples=[]
            for _ in range(4):
                midpoint=yaw+direction*ds*curvature/2
                x+=direction*ds*np.cos(midpoint);y+=direction*ds*np.sin(midpoint)
                old=yaw.copy()
                yaw+=direction*ds*curvature
                x+=self.geometry.axle_offset*(np.cos(yaw)-np.cos(old))
                y+=self.geometry.axle_offset*(np.sin(yaw)-np.sin(old))
                safe&=self.geometry.clear(x,y,yaw,obs,inflation)
                samples.append(np.stack((x.copy(),y.copy()),axis=1))
            for i in np.flatnonzero(safe):
                end=(x[i],y[i],self.wrap(yaw[i]));n=key(end)
                if n==node:continue
                deviation=np.min(np.linalg.norm(reference-np.array(end[:2]),axis=1))
                if local_corridor and deviation>.55:continue
                clearance=np.min(np.linalg.norm(obs[:,:2]-np.array(end[:2]),axis=1)) if len(obs) else 10.
                edge=min(end[0],5-end[0],end[1],2-end[1]) if not local_corridor else .65
                soft=(.5*math.exp(-max(0.,clearance-.30)/.16)+.25*min(float(deviation),.8)
                      +1.4*math.exp(-max(0.,edge-.20)/.15))
                previous=states[came[node]][2] if node in came else pose[2]
                previous_turn=self.wrap(p[2]-previous)/length
                new=cost+length*(1+soft+.025*curvature[i]**2+.04*(curvature[i]-previous_turn)**2)
                if new+1e-9>=costs.get(n,math.inf):continue
                costs[n]=new;states[n]=end;came[n]=node
                segments[n]=[tuple(s[i]) for s in samples]
                heappush(queue,(new+1.8*heuristic(end),new,next(serial),n))
        self.reason='NO_KINEMATIC_CORRIDOR';return []
