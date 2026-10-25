"""ROS-free full-course navigation, using only supplied pose and lidar points."""
import math
import time
from concurrent.futures import ThreadPoolExecutor,ProcessPoolExecutor
import multiprocessing
from dataclasses import dataclass

from .race_course import FRONT, REAR, HALF_WIDTH, legal_pose, wrap, resample, path_yaw
from .race_mpc import SamplingMPC, CURVATURE_GAIN,body_path_headings
from .hybrid_corridor import HybridCorridor
from .frenet_corridor import smooth_ring_corridor
import numpy as np
from .race_tuning import tuning_for

_PLANNING_POOL=ThreadPoolExecutor(max_workers=1,thread_name_prefix='cone-corridor')
_PROCESS_PLANNING_POOL=None


def shutdown_planners():
    """Release workers when an embedding application exits."""
    global _PROCESS_PLANNING_POOL
    if _PROCESS_PLANNING_POOL is not None:
        _PROCESS_PLANNING_POOL.shutdown(wait=False,cancel_futures=True)
        _PROCESS_PLANNING_POOL=None
    _PLANNING_POOL.shutdown(wait=False,cancel_futures=True)


def _planner_ready():return True


def _process_pool():
    global _PROCESS_PLANNING_POOL
    if _PROCESS_PLANNING_POOL is None:
        _PROCESS_PLANNING_POOL=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
        _PROCESS_PLANNING_POOL.submit(_planner_ready)
    return _PROCESS_PLANNING_POOL


def _plan_in_process(stage,continuation,tuning,nominal_index,pose,points,preferred,now,boundary_margin,obstacle_margin):
    follower=CourseFollower(stage,continuation,False,tuning)
    follower.set_boundary_margin(boundary_margin)
    follower.hybrid.geometry.obstacle_margin=obstacle_margin
    follower.nominal_index=nominal_index
    return follower.plan_corridor(pose,points,preferred,now)


def clamp(v,lo,hi):return max(lo,min(hi,v))


@dataclass
class Control:
    speed: float=0.
    steering: float=0.
    mode: str='WAIT_SENSOR'
    remaining: float=0.


def distances(points):
    s=[0.]
    for a,b in zip(points,points[1:]):s.append(s[-1]+math.dist(a,b))
    return s


def project(pose,points,s,index=0):
    best=(1e6,0,0.,0.)
    for i in range(max(0,index-6),min(len(points)-1,index+65)):
        a,b=points[i],points[i+1]
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=clamp(((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/max(dx*dx+dy*dy,1e-12),0.,1.)
        d=math.hypot(pose[0]-a[0]-t*dx,pose[1]-a[1]-t*dy)
        if d<best[0]:best=(d,i,s[i]+t*(s[i+1]-s[i]),t)
    return best


def at_distance(points,s,value):
    from bisect import bisect_right
    i=clamp(bisect_right(s,value)-1,0,len(points)-2)
    t=clamp((value-s[i])/max(s[i+1]-s[i],1e-12),0.,1.)
    return (points[i][0]+t*(points[i+1][0]-points[i][0]),points[i][1]+t*(points[i+1][1]-points[i][1]))


class CourseFollower:
    def __init__(self,stage,continuation=(),background_planning=False,tuning=None):
        self.tuning=tuning or tuning_for()
        self.stage=stage
        self.nominal=stage.points
        self.nominal_s=distances(self.nominal)
        self.points=self.nominal
        self.s=self.nominal_s
        self.index=self.nominal_index=0
        self.last_steer=0.
        self.last_replan=-100.
        self.replans=0
        self.replan_failures=0
        self.control_blocked=0
        self.replanned=False
        self.retained_plans=0
        self.planning_history=[]
        self.background_planning=background_planning
        self.planning_future=None
        self.planned_fitted_cones=0
        self.terminal_latched=False
        self.local_corridor=stage.name.startswith(('C_RING_','ENTER_C','EXIT_C','ENTER_B','EXIT_B'))
        boundary_margin=.025 if self.local_corridor else .035
        self.hybrid=HybridCorridor(extra_boundary=0. if self.local_corridor else .015,boundary_margin=boundary_margin)
        if self.tuning.sprint and not self.local_corridor:
            self.hybrid.geometry.obstacle_margin=.060
        if self.tuning.sprint:
            self.hybrid.time_budget=.08 if self.local_corridor else (3. if stage.name=='RETURN_A_AND_PARK' else .8)
            self.hybrid.analytic_connections=True
            self.hybrid.analytic_approach=stage.direction<0
            self.hybrid.geometry.axle_offset=.0715
            self.hybrid.curvature_limit=3.3 if stage.direction>0 else 3.7
        # Preserve the final tangent, with a longer lead for reversing cusps.
        # A long fixed parking tail can itself be obstructed by return-leg cones.
        tail_length=.55 if stage.direction<0 else .28
        self.tail_index=max(1,next((i for i,v in enumerate(self.nominal_s)
            if v>=self.nominal_s[-1]-tail_length),len(self.nominal)-2))
        # The heading-lattice connector already enforces the final tangent.
        # A fixed earlier anchor can be inside a cone's inflated base even when
        # a wide route to the actual stage endpoint exists.
        if stage.replan:self.tail_index=len(self.nominal)-1
        self.inflation=.20/math.sqrt(2)-(.355*.20*(1-(1-.22)*(.23-.015)/(.30-.015)))+.025
        self.blocked_reason=''
        self.continuation=list(continuation)
        self.mpc=SamplingMPC(boundary_margin=boundary_margin,tuning=self.tuning)
        if self.tuning.sprint and not self.local_corridor:
            self.mpc.progress_weight=8.
            self.mpc.heading_weight=8.

    def set_boundary_margin(self,margin):
        for geometry in (self.mpc,self.hybrid.geometry):
            change=margin-geometry.boundary_margin
            geometry.u+=np.sign(geometry.u)*change
            geometry.w+=np.sign(geometry.w)*change
            geometry.boundary_margin=margin

    def update_ring_margin(self,points):
        if not self.tuning.sprint or not self.stage.name.startswith('C_RING_'):return
        route=np.asarray(self.nominal);tight=False
        for px,py,*_ in getattr(points,'boxes',points):
            distance=np.linalg.norm(route-np.array([px,py]),axis=1);i=int(np.argmin(distance))
            ahead=self.nominal_s[i]-self.nominal_s[self.nominal_index]
            if distance[i]<.35 and -.35<ahead<2.0:tight=True;break
        # The physical body constraint remains hard. Use a smaller additional
        # field reserve for the measured 0.5m cone passage; cone padding stays.
        self.set_boundary_margin(.015 if tight else .025)
        self.hybrid.geometry.obstacle_margin=.020 if tight else .010

    def clear(self,pose,points):
        if hasattr(points,'boxes'):
            return bool(self.mpc.clear(np.array([pose[0]]),np.array([pose[1]]),np.array([pose[2]]),points.boxes,self.inflation)[0])
        if not legal_pose(pose):return False
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        for px,py in points:
            dx,dy=px-x,py-y
            forward,lateral=c*dx+s*dy,-s*dx+c*dy
            if -REAR-self.inflation < forward < FRONT+self.inflation and abs(lateral)<HALF_WIDTH+self.inflation:
                return False
        return True

    def rollout(self,pose,v,speed,steer,points,horizon=.65):
        x,y,yaw=pose
        cv,delta=v,self.last_steer
        for _ in range(math.ceil(horizon/.08)):
            dv=clamp(speed-cv,-.075,.075)
            cv+=dv
            delta+=clamp(steer-delta,-.17,.17)
            yaw=wrap(yaw+cv*math.tan(delta)/.143*.08)
            x+=cv*math.cos(yaw)*.08;y+=cv*math.sin(yaw)*.08
            if not self.clear((x,y,yaw),points):return None
        return x,y,yaw

    def route_blocked(self,pose,route,arc,index,points):
        _,idx,progress,_=project(pose,route,arc,index)
        heading=getattr(route,'body_yaws',None)
        if heading is None and self.tuning.sprint:heading=body_path_headings(route,self.stage.direction)
        # Include the whole currently executable corridor, not just its center.
        for i in range(idx,len(route),2):
            if arc[i]-progress>(3.5 if self.tuning.sprint else 2.0):break
            if not self.clear((*route[i],heading[i] if heading is not None else path_yaw(route,i,self.stage.direction)),points):return True
        return False

    def needs_replan(self,pose,points):
        _,idx,progress,_=project(pose,self.nominal,self.nominal_s,self.nominal_index)
        self.nominal_index=max(self.nominal_index,idx)
        if self.tuning.sprint and self.stage.direction<0 and not self.replanned and self.replans==0:return True
        if progress>=self.nominal_s[self.tail_index]-.15:return False
        if self.control_blocked>=8:return True
        return self.route_blocked(pose,self.points,self.s,self.index,points)

    def plan_corridor(self,pose,points,preferred,now):
        end_index=self.tail_index
        if self.local_corridor:
            _,idx,progress,_=project(pose,self.nominal,self.nominal_s,self.nominal_index)
            end_index=next((i for i in range(idx,len(self.nominal)) if self.nominal_s[i]>=progress+3.0),len(self.nominal)-1)
        goal=self.nominal[end_index]
        goal_yaw=path_yaw(self.nominal,end_index,self.stage.direction)
        begin=time.perf_counter()
        guide=self.nominal[max(0,self.nominal_index-4):end_index+1] if self.local_corridor else preferred
        route=[]
        if self.local_corridor:
            _,idx,_,_=project(pose,self.nominal,self.nominal_s,self.nominal_index)
            smooth_end=len(self.nominal)-1 if self.stage.name.startswith('C_RING_') else end_index
            max_offset=.35 if self.stage.name.startswith(('ENTER_B','EXIT_B')) else .122
            route=smooth_ring_corridor(pose,self.nominal[idx:smooth_end+1],points,self.hybrid.geometry,self.inflation,max_offset=max_offset,
                rear_frame=self.tuning.sprint and self.stage.name.startswith('C_RING_'))
            if route:
                end_index=smooth_end
                self.hybrid.reason='FRENET_CORRIDOR_FOUND';self.hybrid.expansions=0
        adjusted_goal=goal
        if not route and self.tuning.sprint and self.stage.name.startswith('C_RING_'):
            return {'time':now,'pose':list(pose),'reason':'RING_CORRIDOR_NOT_READY',
                    'milliseconds':(time.perf_counter()-begin)*1000,'expansions':0,'route':[],
                    'join_index':end_index,'nominal_goal':goal,'planned_goal':goal}
        if not route:
            goals=[goal]
            obs=np.asarray(getattr(points,'boxes',points),dtype=float)
            if not obs.size:obs=np.empty((0,2))
            if self.tuning.sprint and self.stage.direction<0 and not self.hybrid.geometry.clear(
                np.array([goal[0]]),np.array([goal[1]]),np.array([goal_yaw]),obs,self.inflation,recovery=False)[0]:
                # A cone may cover the original reverse/forward junction.
                # Capture a clear lateral junction in A, preserving heading;
                # the following B corridor then joins from that real pose.
                normal=(-math.sin(goal_yaw),math.cos(goal_yaw))
                goals=[(goal[0]+offset*normal[0],goal[1]+offset*normal[1])
                       for offset in (.30,.45,.18,-.18,-.30,-.45)]
                if self.stage.name=='REVERSE_TRANSFER_A':
                    goals=[p for p in goals if 2.20<=p[0]<=2.80]
            for adjusted_goal in goals:
                route=self.hybrid.plan(pose,adjusted_goal,goal_yaw,points,self.inflation+(0. if self.local_corridor else .04),guide,self.stage.direction,self.local_corridor)
                if route:break
        return {'time':now,'pose':list(pose),'reason':self.hybrid.reason,
            'milliseconds':(time.perf_counter()-begin)*1000,'expansions':self.hybrid.expansions,'route':route,'join_index':end_index,
            'nominal_goal':goal,'planned_goal':adjusted_goal}

    def try_replan(self,pose,points,now,v=0.):
        if self.planning_future is not None:
            if not self.planning_future.done():return False
            result=self.planning_future.result();self.planning_future=None
            self.last_replan=now
            self.accept_plan(result,points)
            return False
        fitted=getattr(points,'fitted',0)
        improved_fit=self.blocked_reason and fitted>self.planned_fitted_cones
        interval=.10 if self.tuning.sprint and self.local_corridor and self.blocked_reason else .7
        if not self.stage.replan or (now-self.last_replan<interval and not improved_fit):return
        if not self.needs_replan(pose,points):
            if self.replanned:self.retained_plans+=1
            return
        self.last_replan=now
        self.planned_fitted_cones=fitted
        preferred=self.points[self.index:] if self.replanned else self.nominal
        self.replans+=1
        if self.background_planning:
            if self.tuning.sprint:
                self.planning_future=_process_pool().submit(_plan_in_process,self.stage,self.continuation,
                    self.tuning,self.nominal_index,tuple(pose),points,list(preferred),now,
                    self.mpc.boundary_margin,self.hybrid.geometry.obstacle_margin)
            else:self.planning_future=_PLANNING_POOL.submit(self.plan_corridor,tuple(pose),points,list(preferred),now)
            return False
        self.accept_plan(self.plan_corridor(pose,points,preferred,now),points)

    def accept_plan(self,result,points):
        self.planning_history.append(result)
        route=result['route']
        if route and not self.clear_route(route,points):
            route=[];result['reason']='UPDATED_CORRIDOR_BLOCKED'
        if route:
            tail=self.nominal[result.get('join_index',self.tail_index)+1:]
            joined=route+tail if tail else route
            if not self.clear_route(joined,points):
                route=[];result['reason']='REJOIN_REFERENCE_BLOCKED'
        if route:
            tail=self.nominal[result.get('join_index',self.tail_index)+1:]
            route=route+tail if tail else route
            # Remove grid heading jumps before computing curvature/speed limits.
            # Accept rounding only when the complete oriented body remains clear.
            rounded=route
            for _ in range(0 if hasattr(route,'body_yaws') else 2):
                cut=[rounded[0]]
                for a,b in zip(rounded,rounded[1:]):
                    cut.extend([(.75*a[0]+.25*b[0],.75*a[1]+.25*b[1]),
                                (.25*a[0]+.75*b[0],.25*a[1]+.75*b[1])])
                rounded=cut+[rounded[-1]]
            rounded=resample(rounded,.025) if not hasattr(route,'body_yaws') else route
            if self.clear_route(rounded,points):
                route=rounded
            self.points=route
            self.s=distances(self.points)
            self.index=0
            self.replanned=True
            self.blocked_reason=''
            self.control_blocked=0
        else:
            self.replan_failures+=1
            self.blocked_reason=result['reason']

    def clear_route(self,route,points):
        p=np.asarray(route)
        yaw=getattr(route,'body_yaws',None)
        if yaw is None:yaw=body_path_headings(route,self.stage.direction) if self.tuning.sprint else np.array([path_yaw(route,i,self.stage.direction) for i in range(len(route))])
        obstacles=np.asarray(getattr(points,'boxes',points),dtype=float)
        if not obstacles.size:obstacles=np.empty((0,2))
        return bool(np.all(self.mpc.clear(p[:,0],p[:,1],yaw,obstacles,self.inflation,recovery=False)))

    def command(self,pose,v,points,now,must_stop=False,stop_after=None,actual_steering=None,steering_rate=0.):
        if not all(math.isfinite(z) for z in (*pose,v)):
            return Control(mode='INVALID_STATE')
        if not legal_pose(pose,margin=0.):
            return Control(mode='OUTSIDE_LEGAL_COURSE')
        self.update_ring_margin(points)
        if self.try_replan(pose,points,now,v):
            self.mpc.command_rate=0.
            return Control(mode='PLAN_CORRIDOR')
        if self.blocked_reason and not self.local_corridor and not self.tuning.sprint and self.needs_replan(pose,points):
            self.mpc.command_rate=0.
            return Control(mode='NO_SAFE_ACTION')
        distance,idx,progress,_=project(pose,self.points,self.s,self.index)
        self.index=max(self.index,idx)
        remaining=max(0.,self.s[-1]-progress)
        end_distance=math.dist(pose[:2],self.points[-1])
        if self.terminal_latched:
            return Control(mode='BRAKE_AT_CUSP',remaining=remaining)
        # Reverse junctions have room for a controlled capture region in A.
        # Latch braking before the real Ackermann chassis overshoots the cusp;
        # never circle around trying to hit a zero-length waypoint precisely.
        reverse=self.stage.direction<0
        capture=.20 if reverse or not must_stop else .11
        if remaining<(.11 if reverse else .055) and end_distance<capture:
            if must_stop:self.terminal_latched=True
            return Control(mode='STAGE_REACHED',remaining=remaining)
        if distance>.55:return Control(mode='ROUTE_LOST',remaining=remaining)
        tracking=self.points+self.continuation
        tracking_s=distances(tracking)
        look=(.34 if reverse else .24)+.14*abs(v)
        target=at_distance(tracking,tracking_s,progress+look)
        dx,dy=target[0]-pose[0],target[1]-pose[1]
        lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
        nominal=clamp(math.atan2(2*.143*lateral,max(dx*dx+dy*dy,.025)),-.52,.52)
        speed_cap=self.stage.max_speed/(1+1.3*abs(nominal))
        if self.tuning.sprint:
            speed_cap=min(self.tuning.maximum_speed,self.stage.max_speed*1.8)
        if self.local_corridor and self.blocked_reason:
            # Do not consume the lane-change distance while waiting for enough
            # scan returns to resolve a narrow gap. New fits bypass plan delay.
            if self.tuning.sprint:
                # Continue only the collision-checked prefix; brake before the
                # first obstructed reference section instead of crawling for
                # the entire time a more distant corridor is being computed.
                p=np.asarray(self.points[self.index:]);a=np.array(self.s[self.index:])-progress
                yaw=body_path_headings(self.points,self.stage.direction)[self.index:]
                obs=np.asarray(getattr(points,'boxes',points),dtype=float)
                if not obs.size:obs=np.empty((0,2))
                ok=self.mpc.clear(p[:,0],p[:,1],yaw,obs,self.inflation,recovery=False)
                ahead=a[~ok];gap=max(0.,float(ahead[0])-.25) if len(ahead) else 3.
                speed_cap=min(speed_cap,.65,max(.12,math.sqrt(2*self.tuning.braking*gap)))
            else:speed_cap=min(speed_cap,.08)
        if self.stage.name.startswith('C_RING_'):
            speed_cap=min(speed_cap,self.tuning.ring_speed)
            if self.tuning.sprint and len(points):
                # Tight C passes have only centimetres of lateral room. Settle
                # steering before the gap while continuing to roll, rather
                # than arriving fast and needing zero-speed pre-steering.
                route=np.asarray(self.nominal)
                c,s=math.cos(pose[2]),math.sin(pose[2])
                for px,py,*_ in getattr(points,'boxes',points):
                    if ((px-pose[0])*c+(py-pose[1])*s>-.10 and
                        math.hypot(px-pose[0],py-pose[1])<1.20 and
                        np.min(np.linalg.norm(route-np.array([px,py]),axis=1))<.35):
                        speed_cap=min(speed_cap,.55);break
        elif self.stage.name.startswith(('ENTER_B','EXIT_B')):
            speed_cap=min(speed_cap,self.tuning.channel_speed)
        # Cone corridors need time for steering response and new laser returns.
        # Prioritize continuous passage over the former 40-second speed target.
        if self.replanned and not self.tuning.sprint:
            c,s=math.cos(pose[2]),math.sin(pose[2])
            nearest=min((math.dist(pose[:2],p) for p in points if (p[0]-pose[0])*c+(p[1]-pose[1])*s>-.10),default=10.)
            speed_cap=min(speed_cap,.65 if nearest>.75 else (.28 if self.local_corridor else .45))
        # Limit lateral acceleration and brake before the upcoming curvature.
        curvature=abs(math.tan(nominal)/.143)
        speed_cap=min(speed_cap,math.sqrt((2.2 if self.tuning.sprint else 1.10)/max(curvature,.01)))
        for ahead in (.15,.30,.50,.75):
            a=at_distance(tracking,tracking_s,progress+ahead-.06)
            b=at_distance(tracking,tracking_s,progress+ahead)
            c=at_distance(tracking,tracking_s,progress+ahead+.06)
            ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
            k=2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/max(ab*bc*ac,1e-8)
            lateral_limit=self.tuning.ring_lateral_acceleration if self.local_corridor else self.tuning.lateral_acceleration
            lead=.15+.30*abs(v) if self.tuning.sprint else .15
            speed_cap=min(speed_cap,math.sqrt(lateral_limit/max(k,.01)+2*self.tuning.braking*max(0.,ahead-lead)))
        if self.stage.name=='RETURN_A_AND_PARK' and remaining<.45:
            speed_cap=min(speed_cap,.60 if self.tuning.sprint else .35)
        if must_stop or stop_after is not None:
            braking_distance=remaining+(stop_after or 0.)-.05-.16*abs(v)
            speed_cap=min(speed_cap,max(.07,math.sqrt(2*self.tuning.braking*max(0.,braking_distance))))
        sign=self.stage.direction
        gain=.7 if sign>0 else .8
        self.mpc.tracking_only=self.tuning.sprint and self.planning_future is not None and not self.replanned
        best=self.mpc.solve(pose,v,gain*math.tan(self.last_steer) if actual_steering is None else actual_steering,
            tracking,self.index,points,self.inflation,speed_cap,sign,must_stop or stop_after is not None,
            steering_rate,self.last_steer)
        if best is None:
            self.control_blocked+=1
            self.mpc.command_rate=0.
            return Control(mode='PLAN_CORRIDOR' if self.planning_future is not None else 'NO_SAFE_ACTION',remaining=remaining)
        speed,steer=best
        if self.tuning.sprint and abs(speed)<.04 and abs(v)<.04 and remaining>.35:
            self.control_blocked+=1
        else:self.control_blocked=0
        self.last_steer=steer
        return Control(speed,steer,'MPC_AVOID_FOLLOW' if self.replanned else 'MPC_PATH_FOLLOW',remaining)


class RaceMission:
    def __init__(self,stages,background_planning=False,profile='balanced',tuning=None):
        self.tuning=tuning or tuning_for(profile)
        self.stages=stages
        self.background_planning=background_planning
        self.index=0
        self.follower=self.make_follower()
        self.events=[]
        self.finished=False
        self.failure=''
        self.stop_since=None
        self.blocked_since=None
        self.all_replans=0
        self.completed_plans=[]
        self.curvature_state=0.;self.curvature_rate=0.;self.state_time=None
        self.motion_mark_pose=None;self.motion_mark_time=0.
        self.prepared_follower=None;self.prepared_index=-1
        if self.tuning.sprint and self.background_planning:_process_pool()

    @property
    def stage(self):
        return self.stages[self.index]

    def make_follower(self,index=None):
        index=self.index if index is None else index
        current=self.stages[index]
        continuation=[];distance=0.;last=current.points[-1]
        for stage in self.stages[index+1:]:
            if stage.direction!=current.direction:break
            for point in stage.points[1:]:
                distance+=math.dist(last,point);last=point
                continuation.append(point)
                if distance>=2.:break
            if distance>=2.:break
        return CourseFollower(current,continuation,self.background_planning,self.tuning)

    def prepare_critical_route(self,points,now):
        if not self.tuning.sprint or not self.background_planning:return
        if self.prepared_follower is not None:
            follower=self.prepared_follower
            start=(*follower.nominal[0],path_yaw(follower.nominal,0,follower.stage.direction))
            follower.try_replan(start,points,now)
            return
        if self.stage.name not in ('TASK_AREA_APPROACH','EXIT_C_RING','EXIT_B_CHANNEL'):return
        name='REVERSE_TRANSFER_A' if self.stage.name=='TASK_AREA_APPROACH' else 'RETURN_A_AND_PARK'
        target=next((i for i in range(self.index+1,len(self.stages))
                     if self.stages[i].name==name),None)
        if target is None:return
        follower=self.make_follower(target)
        start=(*follower.nominal[0],path_yaw(follower.nominal,0,follower.stage.direction))
        follower.try_replan(start,points,now)
        self.prepared_follower=follower;self.prepared_index=target

    @property
    def total_replans(self):
        # A finished final stage has already been included by advance().
        return self.all_replans+(0 if self.finished else self.follower.replans)

    def advance(self,now,pose):
        self.events.append({'event':'STAGE_COMPLETED','stage':self.stage.name,'time':now,'pose':list(pose)})
        self.all_replans+=self.follower.replans
        self.completed_plans.extend(self.follower.planning_history)
        if self.index==len(self.stages)-1:
            self.finished=True
            return
        previous=self.follower
        same_direction=self.stages[self.index+1].direction==self.stage.direction
        self.index+=1
        if self.prepared_index==self.index:
            self.follower=self.prepared_follower
            self.prepared_follower=None;self.prepared_index=-1
        else:self.follower=self.make_follower()
        if same_direction:
            self.follower.last_steer=previous.last_steer
            self.follower.mpc.command_rate=previous.mpc.command_rate
        self.stop_since=self.blocked_since=None

    def command(self,pose,v,points,now,yaw_rate=None,actual_curvature=None):
        if self.finished:return Control(mode='FINISHED')
        if self.failure:return Control(mode=self.failure)
        self.prepare_critical_route(points,now)
        if self.tuning.sprint:
            if self.motion_mark_pose is None or math.dist(pose[:2],self.motion_mark_pose)>.025:
                self.motion_mark_pose=tuple(pose[:2]);self.motion_mark_time=now
            elif now-self.motion_mark_time>6. and abs(v)<.04:
                self.failure='NO_SAFE_ACTION';return Control(mode=self.failure)
        must_stop=(self.index==len(self.stages)-1 or self.stages[self.index+1].direction!=self.stage.direction)
        stop_after=None
        if self.index+2<len(self.stages) and self.stages[self.index+1].direction==self.stage.direction and self.stages[self.index+2].direction!=self.stage.direction:
            stop_after=distances(self.stages[self.index+1].points)[-1]
        actual_steering=None
        if yaw_rate is not None:
            dt=now-self.state_time if self.state_time is not None else .05
            if dt>.001:
                # The predictor state is body yaw response, including tyre
                # understeer. A joint angle is geometric curvature and cannot
                # be substituted at unit gain while the chassis is moving.
                if abs(v)>.08:
                    measured=clamp(.143*yaw_rate/v,-.75,.75)
                elif actual_curvature is not None:
                    measured=actual_curvature if self.tuning.sprint else (.70 if self.stage.direction>0 else .80)*actual_curvature
                else:measured=self.curvature_state*math.exp(-dt/.30)
                derivative=clamp((measured-self.curvature_state)/dt,-3.,3.)
                self.curvature_rate=.55*self.curvature_rate+.45*derivative
                self.curvature_state=measured;self.state_time=now
            actual_steering=self.curvature_state
        cmd=self.follower.command(pose,v,points,now,must_stop,stop_after,actual_steering,self.curvature_rate)
        if cmd.mode in ('STAGE_REACHED','BRAKE_AT_CUSP'):
            if not must_stop:
                self.advance(now,pose)
                return self.command(pose,v,points,now,yaw_rate,actual_curvature)
            if abs(v)<.025:
                if self.stop_since is None:self.stop_since=now
                if now-self.stop_since>(.10 if self.tuning.sprint else .25):
                    self.advance(now,pose)
                    return self.command(pose,v,points,now,yaw_rate,actual_curvature)
            else:self.stop_since=None
            return Control(mode='PARK_BRAKE' if self.index==len(self.stages)-1 else 'DIRECTION_CHANGE_BRAKE')
        blocked_planning=cmd.mode=='PLAN_CORRIDOR' and bool(self.follower.blocked_reason)
        if cmd.mode in ('NO_SAFE_ACTION','ROUTE_LOST','OUTSIDE_LEGAL_COURSE','INVALID_STATE') or blocked_planning:
            if self.blocked_since is None:self.blocked_since=now
            if cmd.mode in ('OUTSIDE_LEGAL_COURSE','INVALID_STATE') or now-self.blocked_since>12:
                self.failure='NO_SAFE_ACTION' if blocked_planning else cmd.mode
        else:self.blocked_since=None
        return cmd
