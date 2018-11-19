#!/usr/bin/env python
import math

#this file is just for testing python stuff

def theta_range_confinement(theta):
  if theta > math.pi:
    return -2*math.pi + theta
  elif theta < -1*math.pi:
    return 2*math.pi + theta
  return theta

print theta_range_confinement(-11*math.pi/6)
