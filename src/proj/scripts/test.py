#!/usr/bin/env python
import math
from itertools import permutations

#this file is just for testing python stuff

def theta_range_confinement(theta):
  if theta > math.pi:
    return -2*math.pi + theta
  elif theta < -1*math.pi:
    return 2*math.pi + theta
  return theta

tup=(1,2,3)
a=list(tup)
print a.insert(0,0)

