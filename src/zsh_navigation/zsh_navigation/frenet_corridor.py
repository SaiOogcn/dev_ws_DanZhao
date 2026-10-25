"""Smooth side corridors along the prescribed ring traversal, without shortcuts."""
import math
import numpy as np
from .race_mpc import body_path_headings


class CorridorPath(list):
    def __init__(self,points,headings):
        super().__init__(points);self.body_yaws=np.asarray(headings)


def smooth_ring_corridor(pose,reference,obstacles,geometry,inflation,max_offset=.122,rear_frame=False):
    route=np.asarray(reference,dtype=float)
    start=np.asarray(pose[:2])-.0715*np.array([math.cos(pose[2]),math.sin(pose[2])]) if rear_frame else np.asarray(pose[:2])
    if len(route)<4:return []
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    dx=np.gradient(route[:,0],arc);dy=np.gradient(route[:,1],arc)
    yaw=np.unwrap(np.arctan2(dy,dx));normal=np.c_[-np.sin(yaw),np.cos(yaw)]
    obs=np.asarray(getattr(obstacles,'boxes',obstacles),dtype=float)
    if not len(obs):return []
    blocked=~geometry.clear(route[:,0],route[:,1],yaw,obs,inflation)
    if not np.any(blocked):return [tuple(p) for p in route]
    distances=np.linalg.norm(route[:,None,:]-obs[None,:,:2],axis=2)
    nearest=np.argmin(distances,axis=0)
    centers=[arc[i] for j,i in enumerate(nearest) if distances[i,j]<.40 and arc[i]>.10]
    if not centers:return []
    best=None
    # The C lane may leave only a centimetre-wide feasible centre corridor.
    # A 2cm lateral lattice skips that corridor even when the body fits.
    spacing=.0025 if max_offset<.13 else .01
    offsets=[sign*value for value in np.arange(.06,max_offset,spacing) for sign in (-1,1)]
    for offset,return_length in ((o,length) for o in offsets for length in (.55,.70,.90)):
        amount=np.zeros(len(route))
        for center in centers:
            ramp=np.where(arc<center,1.0,return_length)
            plateau=np.where(arc<center,.40,.20) if rear_frame else np.where(arc<center,.36,.25)
            t=np.clip((np.abs(arc-center)-plateau)/ramp,0.,1.)
            bump=1-(6*t**5-15*t**4+10*t**3)
            amount=np.maximum(amount,bump)
        profile=offset*amount
        initial_lateral=float(np.dot(start-route[0],normal[0]))
        if initial_lateral*offset>0:
            # A vehicle already on the chosen side must not first return to
            # the middle of the lane and then move aside again. Carry its
            # existing offset into the smooth obstacle corridor.
            t=np.clip(arc/2.,0.,1.)
            fade=1-(6*t**5-15*t**4+10*t**3)
            profile+=initial_lateral*(1-amount)*fade
        trial=route+profile[:,None]*normal
        # Match the real position smoothly, never teleport the path start.
        mismatch=start-trial[0]
        if rear_frame:
            # Match position AND tangent. Position-only blending could demand
            # an immediate opposite turn and reject the short C entry window.
            length=.70;t=np.clip(arc/length,0.,1.)
            derivative=np.array([np.gradient(trial[:,0],arc)[0],np.gradient(trial[:,1],arc)[0]])
            tangent_error=np.array([math.cos(pose[2]),math.sin(pose[2])])-derivative
            h00=2*t**3-3*t*t+1;h10=t**3-2*t*t+t
            trial+=h00[:,None]*mismatch[None,:]+length*h10[:,None]*tangent_error[None,:]
        else:
            blend=np.maximum(0.,1-arc/.5)**3
            trial+=mismatch[None,:]*blend[:,None]
        delta=np.diff(trial,axis=0);heading=np.unwrap(np.arctan2(delta[:,1],delta[:,0]))
        lengths=np.linalg.norm(delta,axis=1)
        if np.any(lengths<1e-7):continue
        curvature=np.abs(np.diff(heading))/np.maximum((lengths[:-1]+lengths[1:])/2,1e-6)
        if len(curvature) and np.max(curvature)>(3.3 if rear_frame else 2.70):continue
        if abs(math.atan2(math.sin(heading[0]-pose[2]),math.cos(heading[0]-pose[2])))>.25:continue
        headings=np.r_[heading,heading[-1]]
        if rear_frame:
            trial=trial+.0715*np.c_[np.cos(headings),np.sin(headings)]
        elif geometry.axle_offset:headings=body_path_headings(trial)
        if not np.all(geometry.clear(trial[:,0],trial[:,1],headings,obs,inflation)):continue
        cost=abs(offset)+.02*float(np.sum(np.diff(heading)**2))
        if best is None or cost<best[0]:
            path=CorridorPath([tuple(p) for p in trial],headings) if rear_frame else [tuple(p) for p in trial]
            best=(cost,path)
    return best[1] if best else []
