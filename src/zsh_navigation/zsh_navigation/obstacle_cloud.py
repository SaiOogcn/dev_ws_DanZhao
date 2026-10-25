"""Observed cone-section clustering for narrow C-ring clearances.

Only well-conditioned circular sections become square-base boxes. Uncertain
clusters retain conservative scan-endpoint obstacles. No scene truth is read.
"""
import math
import numpy as np


class ObstacleCloud(list):
    def __init__(self,points,inflation=.14,section_radius=.030,base_half_width=.11):
        super().__init__(points)
        data=np.asarray(points,dtype=float).reshape((-1,2));remaining=np.ones(len(data),dtype=bool)
        near=np.sum((data[:,None,:]-data[None,:,:])**2,axis=2)<.105**2
        boxes=[];self.fitted=0
        while np.any(remaining):
            seed=int(np.flatnonzero(remaining)[0]);remaining[seed]=False;cluster=[seed];queue=[seed]
            while queue:
                i=queue.pop()
                neighbors=np.flatnonzero(remaining&near[i]).tolist()
                remaining[neighbors]=False;cluster.extend(neighbors);queue.extend(neighbors)
            p=data[cluster];center=None
            if len(p)>=5:
                origin=np.mean(p,axis=0);local=p-origin
                a=2*local;b=np.sum(local*local,axis=1)
                estimate,_,_,singular=np.linalg.lstsq(a,b,rcond=None)
                candidate=origin+estimate;radius=np.mean(np.linalg.norm(p-candidate,axis=1))
                residual=np.std(np.linalg.norm(p-candidate,axis=1))
                # Reject strips, sparse glancing hits and non-cone obstacles.
                if len(singular)==2 and singular[1]>.12*singular[0] and .012<radius<.055 and residual<.009:
                    # One visible noisy arc biases a free algebraic circle
                    # towards the scanner. Use the known 20cm cone's ~3cm
                    # section at the 23cm laser height, keeping its full base.
                    # Bound each update so a poor fit remains a raw obstacle.
                    for _ in range(8):
                        d=candidate-p;length=np.maximum(np.linalg.norm(d,axis=1),1e-6)
                        jacobian=d/length[:,None]
                        step=np.linalg.lstsq(jacobian,length-section_radius,rcond=None)[0]
                        candidate-=np.clip(step,-.01,.01)
                    error=np.sqrt(np.mean((np.linalg.norm(p-candidate,axis=1)-section_radius)**2))
                    if error<.009:center=candidate
            if center is not None:
                boxes.append((float(center[0]),float(center[1]),base_half_width,base_half_width));self.fitted+=1
            else:
                boxes.extend((float(x),float(y),inflation,inflation) for x,y in p)
        self.boxes=np.asarray(boxes,dtype=float).reshape((-1,4))
