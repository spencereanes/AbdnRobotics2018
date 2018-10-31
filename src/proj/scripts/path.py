#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import numpy as np
import time

import marker
from PriorityQueue import *

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

"""
To Do:
pre-compute filled squares for time speed up
order dict with tuples rather than hashing myself
figure out how to modify astar for jfs
"""

class path:
  def __init__(self):
    print 'here'
    rospy.init_node('astar_path')
    self.r=rospy.Rate(10)
    self.m=marker.Markers()
    print 'here'
    self.arr=np.empty([2,2])
    print 'here'
    #try:
    self.arr=np.genfromtxt('inflated_data.csv',delimiter=',')
    #except:
      #print "No precomputed occupancygrid, recomputing"
    
    #self.res=self.arr[2]
    self.res=0.012
    self.marr=np.array(self.arr[2:]).reshape(self.arr[0],self.arr[1])

    self.goals=[]
    self.get_goals()


    self.p=self.efficient_path([-4.8,-3.6],True)

    self.draw_path()
    

  def draw_path(self):
    m=marker.Markers("/path")
    for subpath in self.p:
      a=range(len(subpath))
      for i in a[0::10]:
        m.add(subpath[i][0]+6,subpath[i][1]+4.8,1,0,0,'real_robot_pose')
    while not rospy.is_shutdown():
      m.draw()
      self.r.sleep()

  #get goals from parameter server
  def get_goals(self):
    for i in range(0,5):
      pname='/goal%s'%i
      self.goals.append(rospy.get_param(pname))

    
  #could return a value outside the bounds of the array
  def convert_index(self,tup):
    temp=tup[:]
    temp[0]+=6
    temp[1]+=4.8
    return [ int(math.ceil(x / self.res)) for x in temp ]

  def deconvert(self,tup):
    return [round(tup[0]*self.res - 6,3), round(tup[1]*self.res-4.8,3)]

  """
  This method estimates the shortest path not taking obstacles into account.
  It estimates the path using a heuristic of choice.
  """
  def efficient_path(self,rob_pos,timing=False):
    seq = []
    seq.append(rob_pos)
    remaining_goals = list(self.goals)
    while len(remaining_goals) > 0 :
      shortest=remaining_goals[0]
      sdist=self.heur(seq[-1],shortest)
      for g in remaining_goals:
        if self.heur(seq[-1],g) < sdist:
          shortest=g
          sdist=self.heur(seq[-1],g)

      remaining_goals.remove(shortest)
      seq.append(shortest)

    path = []
    start_time = time.time()
    #len(seq)-1
    for i in range(2):
      #reverse index path
      rind_path=self.astar(seq[i],seq[i+1])
      curr=seq[i+1]

      temp=[]
      temp.append(self.hasher(self.convert_index(curr)))
      on=temp[-1]

      while (rind_path[on]) is not None:
        temp.append(rind_path[on])
        on=rind_path[on]


      temp.reverse()
      temp=map(self.dehasher,temp)
      temp=map(self.deconvert,temp)

      path.append(temp)
    
    end_time=time.time()
    rospy.loginfo("\nPath Constructed")
    if timing:
      print "Time (s): ", end_time-start_time
    return path

  """
  #this function expects start and end to be tuples
  #in the value ranges for values in the /map frame
  #i.e. x:[-6,6) ; y:[-4.8,4.8)

  #returns a list of positions 
  #(not sure if I want to use indices or positions yet)
  """
  def astar(self,start,end):
    #start index
    sind=self.convert_index(start)
    #end index
    eind=self.convert_index(end)
    
    q=PriorityQueue()
    cost_so_far={}
    came_from={}
    
    q.put(sind,0)
    
    came_from[self.hasher(sind)]=None
    cost_so_far[self.hasher(sind)]=0

    while not q.empty():
      parent=q.get()
      #note: get returns only the index, not the priority
      if parent == eind:
        break
      #print parent

      for child in self.children(parent):
        #print child
        if not self.is_passable(child):
          continue

        cost1=cost_so_far[self.hasher(parent)]+self.euclidean(parent[0],parent[1],child[0],child[1])
        if self.hasher(child) not in cost_so_far or cost1 < cost_so_far[self.hasher(child)]:
          cost_so_far[self.hasher(child)]=cost1
          new_cost=self.heur(child,eind) + cost1
          q.put(child,new_cost)
          came_from[self.hasher(child)]=self.hasher(parent)

    return came_from

  def hasher(self,index):
    return 1000*index[0]+index[1]

  def dehasher(self,hashed_index):
    return [int(math.floor(hashed_index/1000)),hashed_index%1000]

  def children(self,index):
    return [ [index[0]+1,index[1]], [index[0]-1,index[1]], [index[0],index[1]+1], [index[0],index[1]-1], [index[0]+1,index[1]+1], [index[0]-1,index[1]-1], [index[0]+1,index[1]-1], [index[0]-1,index[1]+1]  ]
  
  
  #deprecated by use of precomputed obstacle inflation
  def is_passable(self,index,dist=1):
    try:
      #print [x,y]
      if self.marr[x,y] == 100:
        #print "inside"
        return False
    except:
      return False
    return True
  
    #what is this
    """
    for x in range(index[0]-dist,index[0]+dist):
      for y in range(index[1]-dist,index[1]+dist):
        try:
          if self.data[x,y] == 100:
            return False
        except:
          return False
    return True
    """

  """
  This function will find the sequence of shortest pathes between the goals.
  This version is much less efficient as it will find the shortest path.
  **This will need to calculate the cost between all nodes and find the 
    smallest combination that reaches all nodes.**
  """
  #def shortest_path(self,rob_pos):
    #for elem in self.goals

  """
  This method finds a somewhat efficient path greedily;
  It finds the closest point to the robot using astar, then the closest remaining point etc.
  """
  #def greedy_path(self,rob_pos):

  #heuristic #1 - taxicab distance between two points assuming no obstacles
  #represents being able to move in four major directions (no diagonals)
  #might look into changing point formatting in the future
  def manhattan(self,x1,y1,x2,y2):
    return (abs(x1-x2) + abs(y1-y2))
    
  #heuristic #2 - euclidean distance between two points
  #represents being able to move in diagonals
  def euclidean(self,x1,y1,x2,y2):
    return (math.sqrt((x1-x2)**2 + (y1-y2)**2))

  #this function is for convenience - change one line here 
  #to alter which hueristic we use in all future functions.
  def heur(self,p1,p2):
    return self.euclidean(p1[0],p1[1],p2[0],p2[1])



do=path()



