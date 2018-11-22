#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import numpy as np
import time

from controller import *
from path import *

#pull path info and create path
p1=path()
#get path
path=p1.get_path()
#send path to controller to move jorge
cont=controller(path,True)#second arg determines whether it gives output
