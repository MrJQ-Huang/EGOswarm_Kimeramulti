"""Pure shared geometry for the ideal EGO planning occupancy baseline.

This known-scene cloud is exclusively for local simulated planning. It is never
an observation or a replacement for Kimera's reconstructed map.
"""
import math
from scene_geometry import points


def solid_axis(low,high,step):
 """Centered lattice with endpoints; gaps are <=step and the center is present."""
 center=(low+high)/2.;count=int(math.floor((high-low)/2./step))
 return sorted({round(v,9) for v in [low,high]+[center+i*step for i in range(-count,count+1)]})


def planning_occupancy_points(description,step):
 """Surface floor + solid boxes/cylinders, only for the oracle EGO input.

 Retain scene_geometry.points' surface contract unchanged for all other users.
 Interior samples prevent a thick closed obstacle shell from leaving its core
 falsely free in EGO's inflated occupancy grid. This is not sensor simulation.
 """
 if not math.isfinite(step) or step<=0:raise ValueError('Planning step must be positive and finite')
 result=set(points(description,step=step))
 def add(x,y,z):result.add((round(x,9),round(y,9),round(z,9)))
 for obstacle in description['obstacles']:
  if obstacle['type']=='box':
   axes=[solid_axis(c-size/2,c+size/2,step) for c,size in zip(obstacle['center'],obstacle['size'])]
   for x in axes[0]:
    for y in axes[1]:
     for z in axes[2]:add(x,y,z)
  elif obstacle['type']=='cylinder':
   x,y=obstacle['center'];radius=obstacle['radius'];bottom=obstacle.get('base_z',0.)
   offsets=solid_axis(-radius,radius,step);heights=solid_axis(bottom,bottom+obstacle['height'],step)
   for dx in offsets:
    for dy in offsets:
     if dx*dx+dy*dy<=radius*radius+1e-12:
      for z in heights:add(x+dx,y+dy,z)
  else:raise ValueError('Unsupported solid planning obstacle '+str(obstacle['type']))
 return sorted(result)
