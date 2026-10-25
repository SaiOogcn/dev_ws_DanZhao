"""Finite-horizon Ackermann control using deterministic cross-entropy shooting.

This is a local CEM sampling MPC, not the Nav2 MPPI plugin. Steering sequences
are parameterized as corrections to a feedback tracking policy. Only the first
action is applied; the complete constrained optimization repeats at 20 Hz.
"""
import math
import numpy as np
from .race_course import FRONT, REAR, HALF_WIDTH, OUTER, INNER
from .race_tuning import tuning_for

# Development-trial value; physical vehicle identification is required.
CURVATURE_GAIN=.70


def steering_transition(damping,frequency2,dt):
    """Exact discretization of the identified second-order steering servo."""
    half=damping/2;disc=frequency2-half*half;decay=math.exp(-half*dt)
    if abs(disc)<1e-10:c=1.;s=dt
    elif disc>0:
        w=math.sqrt(disc);c=math.cos(w*dt);s=math.sin(w*dt)/w
    else:
        w=math.sqrt(-disc);c=math.cosh(w*dt);s=math.sinh(w*dt)/w
    return decay*(c+half*s),decay*s,-frequency2*decay*s,decay*(c-half*s)


def body_path_headings(route,direction=1,axle_offset=.0715):
    """Body yaw for a centre path, including the rear-axle tangent offset."""
    p=np.asarray(route,dtype=float)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    arc=np.maximum.accumulate(arc+np.arange(len(arc))*1e-10)
    tangent=np.unwrap(np.arctan2(np.gradient(p[:,1]),np.gradient(p[:,0])))
    k=np.gradient(tangent,arc)
    return tangent+(math.pi if direction<0 else 0.)-np.arcsin(np.clip(axle_offset*direction*k,-.5,.5))


def rectangle_distance(x,y,b):
    x0,y0,x1,y1,r=b
    dx=np.abs(x-(x0+x1)/2)-(x1-x0)/2+r
    dy=np.abs(y-(y0+y1)/2)-(y1-y0)/2+r
    return np.hypot(np.maximum(dx,0),np.maximum(dy,0))+np.minimum(np.maximum(dx,dy),0)-r


def allowed_points(x,y):
    box=(x>=0)&(x<=5)&(y>=0)&(y<=5)
    ring=(rectangle_distance(x,y,OUTER)<=0)&(rectangle_distance(x,y,INNER)>=0)
    stem=(x>=2)&(x<=3)&(y<=OUTER[1])
    return box&np.where(y<2,True,np.where(y<2.5,(x>=2)&(x<=3),ring|stem))


class SamplingMPC:
    def __init__(self,boundary_margin=.035,tuning=None):
        self.tuning=tuning or tuning_for()
        self.rng=np.random.default_rng(19)
        self.warm=np.array([0.,0.,0.,1.,1.,1.])
        self.last_prediction=[]
        self.last_feasible=0
        self.last_cost=None
        self.command_rate=0.
        self.recovery=False
        self.obstacle_margin=.010
        self.axle_offset=.0715 if self.tuning.sprint else 0.
        self.progress_weight=3.
        self.heading_weight=16.
        self.tracking_only=False
        # Fixed shooting directions avoid cycle-to-cycle Monte-Carlo chatter.
        row=np.arange(48)[:,None]+1;column=np.arange(6)[None,:]+1
        self.directions=np.sin(row*column*2.399963229728653)*1.65
        margin=self.boundary_margin=boundary_margin
        u=np.linspace(-REAR-margin,FRONT+margin,7)
        w=np.linspace(-HALF_WIDTH-margin,HALF_WIDTH+margin,7)
        self.u=np.concatenate((u,u,np.full(7,u[0]),np.full(7,u[-1])))
        self.w=np.concatenate((np.full(7,w[0]),np.full(7,w[-1]),w,w))

    def clear(self,x,y,yaw,obstacles,inflation,recovery=None):
        c,s=np.cos(yaw),np.sin(yaw)
        recover=self.recovery if recovery is None else recovery
        u=self.u-np.sign(self.u)*self.boundary_margin if recover else self.u
        w=self.w-np.sign(self.w)*self.boundary_margin if recover else self.w
        px=x[:,None]+c[:,None]*u-s[:,None]*w
        py=y[:,None]+s[:,None]*u+c[:,None]*w
        valid=np.all(allowed_points(px,py),axis=1)
        if len(obstacles):
            if obstacles.shape[1]==4:
                # SAT: oriented body/bumper rectangle vs observed square bases.
                offset=(FRONT-REAR)/2;length=(FRONT+REAR)/2+self.obstacle_margin;width=HALF_WIDTH+self.obstacle_margin
                dx=obstacles[None,:,0]-(x+c*offset)[:,None]
                dy=obstacles[None,:,1]-(y+s*offset)[:,None]
                hx,hy=obstacles[None,:,2],obstacles[None,:,3]
                ac,ass=np.abs(c[:,None]),np.abs(s[:,None])
                hit=((np.abs(c[:,None]*dx+s[:,None]*dy)<length+hx*ac+hy*ass)&
                     (np.abs(-s[:,None]*dx+c[:,None]*dy)<width+hx*ass+hy*ac)&
                     (np.abs(dx)<hx+length*ac+width*ass)&
                     (np.abs(dy)<hy+length*ass+width*ac))
                return valid&~np.any(hit,axis=1)
            dx=obstacles[None,:,0]-x[:,None]
            dy=obstacles[None,:,1]-y[:,None]
            forward=c[:,None]*dx+s[:,None]*dy
            lateral=-s[:,None]*dx+c[:,None]*dy
            hit=(forward>-REAR-inflation)&(forward<FRONT+inflation)&(np.abs(lateral)<HALF_WIDTH+inflation)
            valid&=~np.any(hit,axis=1)
        return valid

    @staticmethod
    def projection(x,y,route,arc):
        a=route[:-1];d=route[1:]-a
        length2=np.maximum(np.sum(d*d,axis=1),1e-12)
        t=np.clip(((x[:,None]-a[:,0])*d[:,0]+(y[:,None]-a[:,1])*d[:,1])/length2,0,1)
        distance2=(x[:,None]-a[:,0]-t*d[:,0])**2+(y[:,None]-a[:,1]-t*d[:,1])**2
        idx=np.argmin(distance2,axis=1);row=np.arange(len(x))
        progress=arc[idx]+t[row,idx]*(arc[idx+1]-arc[idx])
        return distance2[row,idx],progress,idx

    def evaluate(self,parameters,pose,v,steering,steering_rate,current_input,route,arc,obstacles,inflation,speed_cap,direction,terminal):
        count=len(parameters);dt=.06;steps=24
        x=np.full(count,float(pose[0]));y=np.full(count,float(pose[1]));yaw=np.full(count,float(pose[2]))
        velocity=np.full(count,float(v));delta=np.full(count,float(current_input))
        command_rate=np.full(count,float(self.command_rate))
        curvature_state=np.full(count,float(steering));curvature_rate=np.full(count,float(steering_rate))
        gain,damping,frequency2=(.70,2.62,12.25) if direction>0 else (.80,1.63,6.99)
        if self.tuning.sprint:
            # Open-loop physical identification, including loaded steering
            # servo dynamics. Forward/reverse responses differ substantially.
            gain,damping,frequency2=((self.tuning.forward_gain,self.tuning.forward_damping,self.tuning.forward_frequency2)
                if direction>0 else (self.tuning.reverse_gain,self.tuning.reverse_damping,self.tuning.reverse_frequency2))
        transition=steering_transition(damping,frequency2,dt)
        valid=np.ones(count,dtype=bool);cost=np.zeros(count)
        _,origin,_=self.projection(x,y,route,arc)
        first_v=first_delta=None
        predictions=[]
        path_heading=np.unwrap(np.arctan2(np.gradient(route[:,1]),np.gradient(route[:,0])))
        route_curvature=np.clip(np.gradient(path_heading,arc),-3.,3.)
        for step in range(steps):
            distance2,progress,idx=self.projection(x,y,route,arc)
            look=(.34 if direction<0 else .28)+.20*np.abs(velocity)
            target=np.minimum(arc[-1],progress+look)
            tx=np.interp(target,arc,route[:,0]);ty=np.interp(target,arc,route[:,1])
            dx,dy=tx-x,ty-y
            if self.tuning.sprint:
                # Pose/footprint are centred between the axles. Pure Pursuit
                # and bicycle steering refer to the rear axle, 71.5mm behind.
                target_k=np.interp(target,arc,route_curvature)
                target_body=np.interp(target,arc,path_heading)+(math.pi if direction<0 else 0.)-np.arcsin(np.clip(.0715*direction*target_k,-.5,.5))
                dx-=.0715*(np.cos(target_body)-np.cos(yaw))
                dy-=.0715*(np.sin(target_body)-np.sin(yaw))
            lateral=-np.sin(yaw)*dx+np.cos(yaw)*dy
            base=np.arctan2(2*.143*lateral,gain*np.maximum(dx*dx+dy*dy,.022))
            preview=progress+.28+.20*np.abs(velocity)
            feedforward=np.arctan(.143*direction*np.interp(preview,arc,route_curvature)/gain)
            base=.65*base+.35*feedforward
            ratio=step/(steps-1)*2;left=min(int(ratio),1);fraction=ratio-left
            offset=parameters[:,left]*(1-fraction)+parameters[:,left+1]*fraction
            factor=parameters[:,3+left]*(1-fraction)+parameters[:,4+left]*fraction
            target_delta=np.clip(base+offset,-.52,.52)
            action_dt=.05 if step==0 else dt
            rate=self.tuning.steering_rate;acceleration=self.tuning.steering_acceleration
            desired_rate=np.clip((target_delta-delta)/action_dt,-rate,rate)
            command_rate+=np.clip(desired_rate-command_rate,-acceleration*action_dt,acceleration*action_dt)
            desired=np.clip(delta+command_rate*action_dt,-.52,.52)
            corner_speed=np.sqrt(self.tuning.lateral_acceleration/np.maximum(np.abs(gain*np.tan(desired)/.143),.01))
            planned_speed=np.minimum(speed_cap,corner_speed)
            speed=planned_speed*factor
            if terminal:
                remaining=np.maximum(0,arc[-1]-progress-.045-.13*np.abs(velocity))
                speed=np.minimum(speed,np.sqrt(2*self.tuning.braking*remaining))
                planned_speed=np.minimum(planned_speed,np.sqrt(2*self.tuning.braking*remaining))
                if self.tuning.sprint:speed=planned_speed*factor
            target_v=direction*speed
            wheel_gain=None
            if self.tuning.sprint:
                wheel_gain=1-.60*np.abs(np.tan(desired)) if direction>0 else 1+.20*np.abs(np.tan(desired))
                target_v=direction*np.minimum(self.tuning.maximum_speed,speed/wheel_gain)
            if step==0:first_v=target_v.copy();first_delta=desired.copy()
            # Identified speed/steering lag, including the command adapter rates.
            if self.tuning.sprint:
                target_v*=wheel_gain
            velocity+=np.clip((target_v-velocity)*dt/self.tuning.speed_time_constant,
                -self.tuning.speed_acceleration*dt,self.tuning.speed_acceleration*dt)
            delta+=np.clip(desired-delta,-2.8*dt,2.8*dt)
            # Actual steering has a damped second-order response, not an
            # instantaneous bicycle angle or a first-order command filter.
            if self.tuning.sprint:
                a,b,c,d=transition;target=gain*np.tan(delta);error=curvature_state-target
                next_state=target+a*error+b*curvature_rate
                curvature_rate=c*error+d*curvature_rate;curvature_state=next_state
            else:
                curvature_rate+=(frequency2*(gain*np.tan(delta)-curvature_state)-damping*curvature_rate)*dt
                curvature_state+=curvature_rate*dt
            omega=velocity*curvature_state/.143
            if self.tuning.sprint:
                midpoint=yaw+omega*dt/2
                x+=dt*(velocity*np.cos(midpoint)-.0715*omega*np.sin(midpoint))
                y+=dt*(velocity*np.sin(midpoint)+.0715*omega*np.cos(midpoint))
                yaw+=omega*dt
            else:
                yaw+=omega*dt
                x+=velocity*np.cos(yaw)*dt;y+=velocity*np.sin(yaw)*dt
            valid&=self.clear(x,y,yaw,obstacles,inflation)
            if self.recovery:
                reserve=self.clear(x,y,yaw,obstacles,inflation,recovery=False)
                cost+=dt*8.*~reserve
            tangent=np.arctan2(route[idx+1,1]-route[idx,1],route[idx+1,0]-route[idx,0])
            if direction<0:tangent+=math.pi
            motion_yaw=yaw+np.arctan(.5*curvature_state) if self.tuning.sprint else yaw
            error=np.arctan2(np.sin(motion_yaw-tangent),np.cos(motion_yaw-tangent))
            heading_weight=np.where(terminal&(arc[-1]-progress<.55),24.,self.heading_weight)
            rate_weight=.06 if self.tuning.sprint else .12
            cost+=dt*(200*distance2+heading_weight*error**2+.10*offset**2+rate_weight*command_rate**2)
            if self.tuning.sprint:
                cost+=dt*2.5*(planned_speed-np.abs(velocity))**2
            predictions.append(np.stack((x.copy(),y.copy(),yaw.copy()),axis=1))
        distance2,progress,_=self.projection(x,y,route,arc)
        if self.recovery:valid&=self.clear(x,y,yaw,obstacles,inflation,recovery=False)
        cost+=35*distance2-self.progress_weight*(progress-origin)
        cost+=np.where((np.abs(first_v)<.01)&(arc[-1]-origin>.10),.8 if self.tuning.sprint else .20,0.)
        cost+=.05*np.sum(np.diff(parameters[:,:3],axis=1)**2,axis=1)
        cost[~valid]=np.inf
        return cost,first_v,first_delta,np.stack(predictions,axis=1)

    def solve(self,pose,v,steering,points,index,obstacles,inflation,speed_cap,direction,terminal=False,steering_rate=0.,current_input=0.):
        route=np.asarray(points[max(0,index-3):min(len(points),index+(160 if self.tuning.sprint else 100))],dtype=float)
        if self.tuning.sprint and len(route)>80:
            # Keep the physical preview distance without quadratic projection
            # cost growing with every 2.5cm planner sample.
            route=np.vstack((route[::2],route[-1]))
        if len(route)<2:return None
        lengths=np.linalg.norm(np.diff(route,axis=0),axis=1)
        route=route[np.r_[True,lengths>1e-7]]
        if len(route)<2:return None
        lengths=np.linalg.norm(np.diff(route,axis=0),axis=1)
        arc=np.concatenate(([0.],np.cumsum(lengths)))
        obstacles=np.asarray(getattr(obstacles,'boxes',obstacles),dtype=float)
        if obstacles.size==0:obstacles=np.empty((0,2))
        self.recovery=not bool(self.clear(np.array([pose[0]]),np.array([pose[1]]),np.array([pose[2]]),obstacles,inflation,recovery=False)[0])
        if self.recovery and not bool(self.clear(np.array([pose[0]]),np.array([pose[1]]),np.array([pose[2]]),obstacles,inflation)[0]):return None
        mean=self.warm.copy();std=np.array([.16,.20,.22,.18,.20,.24])
        best=None;feasible=0
        for _ in range(2):
            candidates=mean+self.directions*std
            candidates[:,:3]=np.clip(candidates[:,:3],-.38,.38)
            candidates[:,3:]=np.clip(candidates[:,3:],.22,1.)
            candidates[0]=mean
            candidates[1]=[0,0,0,1,1,1]
            candidates[2]=[0,0,0,.55,.55,.55]
            candidates[3]=[0,0,0,.25,.25,.25]
            # Allow steering to settle before moving, instead of an unrecoverable
            # all-candidates-rejected stop near a tight bend.
            candidates[4]=[0,0,0,0,.65,1]
            candidates[5]=[-.2,-.1,0,0,.65,1]
            candidates[6]=[.2,.1,0,0,.65,1]
            candidates[7]=[0,0,0,0,0,1]
            candidates[8]=[-.3,-.2,0,0,0,1]
            candidates[9]=[.3,.2,0,0,0,1]
            if self.tuning.sprint:
                # Full-horizon braking alternatives also optimize steering;
                # holding the old wheel angle is unsafe on a fast bend.
                candidates[10]=[0,0,0,0,0,0]
                candidates[11]=[-.25,-.15,0,0,0,0]
                candidates[12]=[.25,.15,0,0,0,0]
            if terminal and arc[-1]-arc[min(3,len(arc)-1)]<.25 and abs(v)<.10:
                candidates[:,3:]=1.
            if self.tracking_only:candidates[:,:3]=0.
            costs,vs,deltas,prediction=self.evaluate(candidates,pose,v,steering,steering_rate,current_input,route,arc,obstacles,inflation,speed_cap,direction,terminal)
            safe=np.flatnonzero(np.isfinite(costs));feasible+=len(safe)
            if not len(safe):continue
            if self.tuning.sprint and arc[-1]-arc[min(3,len(arc)-1)]>.35:
                moving=safe[np.abs(vs[safe])>min(.12,.4*speed_cap)]
                # Prefer an actually executable moving trajectory to an idle
                # solution when both satisfy the complete hard constraints.
                if len(moving):safe=moving
            order=safe[np.argsort(costs[safe])]
            winner=order[0]
            if best is None or costs[winner]<best[0]:
                best=(float(costs[winner]),float(vs[winner]),float(deltas[winner]),candidates[winner].copy(),prediction[winner].copy())
            elite=candidates[order[:8]]
            mean=np.mean(elite,axis=0);std=np.maximum(np.std(elite,axis=0),[.035,.04,.05,.04,.05,.06])
        self.last_feasible=feasible
        if best is None:self.last_prediction=[];self.last_cost=None;return None
        value,speed,steer,parameters,prediction=best
        self.command_rate=float(np.clip((steer-current_input)/.05,-self.tuning.steering_rate,self.tuning.steering_rate))
        self.last_prediction=prediction.tolist();self.last_cost=value
        # Shift the previous solution by one 20 Hz command interval.
        for start in (0,3):
            self.warm[start:start+3]=np.interp(np.array([0,.5,1])+.05/1.44,[0,.5,1],parameters[start:start+3])
        return speed,steer
