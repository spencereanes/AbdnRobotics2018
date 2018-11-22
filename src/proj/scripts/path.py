#!/usr/bin/env python
import tf
import rospy
import roslib
import math
import numpy as np
import time
import itertools

import marker
from PriorityQueue import *

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


class path:
  def __init__(self):

    rospy.init_node('astar_path')
    self.r=rospy.Rate(10)

    self.arr=np.empty([2,2])
    self.get_map()
    self.map_width=self.map.info.width
    self.map_height=self.map.info.height
    self.res=self.map.info.resolution

    try:
      self.p=np.genfromtxt('/home/viki/catkin_ws/src/proj/scripts/preplanned_path.csv',delimiter=',')
      self.p=self.p.tolist()
      print "using precomputed path"
    except:
      print "no precomputed path"
      try:
        self.map.data=np.genfromtxt('/home/viki/catkin_ws/src/proj/scripts/inflated_data.csv',delimiter=',')
      except:
        print "No precomputed occupancygrid"

      self.goals=[]
      self.get_goals()
      rospy.loginfo("Goals captured")
      start=rospy.get_param("robot_start")
      self.p=self.shortest_path(start[0:2])
      #this 'flattens' the list of lists into a single list
      self.p = [item for sublist in self.p for item in sublist]
      #print "COMPUTED PATH: ", self.p[1:10]
      np.savetxt("/home/viki/catkin_ws/src/proj/scripts/preplanned_path.csv",self.p,fmt='%.3f',delimiter=",")
      
  def get_path(self):
    return self.p
    
  def indexer(self,i,j):
    return j*1000+i
  
  def get_map(self):
    self.map=rospy.wait_for_message('/map',OccupancyGrid,timeout=None)
    rospy.loginfo('\nMap information collected')
    #print "res: ", self.map.info.resolution
    print "width,height: ", self.map.info.width, self.map.info.height

  def draw_path(self):
    m=marker.Markers("/path")
    for point in self.p:
      m.add(point[0],point[1],1,0,0,'map')
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
    return ( int(math.ceil((tup[0]+6) / self.res)), int(math.ceil((tup[1]+4.8) / self.res))  )

  def deconvert(self,tup):
    return [round(tup[0]*self.res - 6,3), round(tup[1]*self.res-4.8,3)]

  """
  This function will find the sequence of shortest pathes between the goals.
  This version is much less efficient as it will find the shortest path.
  **This will need to calculate the cost between all nodes and find the 
    smallest combination that reaches all nodes.**
  """
  def shortest_path(self,rob_pos):
    seq = []
    seq.append(rob_pos)
    for goal in self.goals:
      seq.append(goal)
    
    cost={}
    rind_path={}
    path_exists={}
    indices = range(len(seq))

    #determine paths between all nodes, and their costs
    for i in range(len(seq)):
      for j in range(i+1,len(seq)):
        print "computing path between ", i, " and ", j
        print seq[i], seq[j]
        out = self.astar(seq[i],seq[j])
        cost[(i,j)]=out[2]
        rind_path[(i,j)]=out[1]
        path_exists[(i,j)]=out[0]
        if not path_exists[(i,j)]:
          print "point ", seq[j], " is unreachable."
          seq.pop(j)
          indices.pop(j)
    
    nodes=list(indices)
    nodes.pop(0)
    print

    #though this is O(n!) time, for a relatively small number of goals,
    #it takes less time than actually computing the paths, and is much simpler
    visit_order=list(itertools.permutations(nodes))
    #this should be more than any possible combination ofpaths
    min_cost = (self.map_width + self.map_height) * (len(indices)+1)+1
    min_order = None
    for order in visit_order:
      print order
      c = cost[0,order[0]]
      for node in range(len(order)-2):
        try:
          c = c + cost[(order[node],order[node+1])]
        except KeyError:
          c = c + cost[(order[node+1],order[node])]
      print "cost for ", order, " is ", c
      if c < min_cost:
        min_cost = c
        min_order = order

    print
    print "min order: ", min_order
    print "min cost: ", min_cost
    min_list=[]
    min_list.append(0)
    for item in min_order:
      min_list.append(item)
      #print seq[item]

    
    path=[]
    for i in range(len(min_list)-1):
      temp=[]
      on=self.convert_index(seq[min_list[i+1]])
      temp.append(on)
      
      #try:
      print "rind_path ", i
      try:
        path_dict=rind_path[(min_list[i],min_list[i+1])]
      except KeyError:
        path_dict=rind_path[(min_list[i+1],min_list[i])]
      if path_dict[on]==None:
        on=self.convert_index(seq[min_list[i]])
      while (path_dict[on]) is not None:
        temp.append(path_dict[on])
        on=path_dict[on]
        #print on
      #except:
        #print "error in path building"

      temp=map(self.deconvert,temp)
      print temp[0]
      #print "SHORTEST PATH temp: ", temp
      path.append(temp)

    path.reverse()
    print "len path: ", len(path)
    return path

  """
  This method estimates the shortest path not taking obstacles into account.
  It estimates the path using a heuristic of choice.
  """
  def efficient_path(self,rob_pos,timing=False):
    seq = [] #ordered goals
    seq.append(rob_pos)
    remaining_goals = list(self.goals)
    while len(remaining_goals) > 0:
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
    i = 0
    ctr = 0
    while ctr < len(seq)-1:
      #reverse index path
      a_star_out=self.astar(seq[i],seq[i+1])
      rind_path=a_star_out[1]
      #print a_star_out[2]
      curr=seq[i+1]
      if not a_star_out[0]:
        seq.pop(i+1)
        #ctr=ctr+1
        continue

      temp=[]
      on=self.convert_index(curr)
      temp.append(on)
      
      try:
        while (rind_path[on]) is not None:
          temp.append(rind_path[on])
          on=rind_path[on]
      except:
        print "error in path building"

      #don't need to reverse, controller wants it in reverse order
      #temp.reverse()
      temp=map(self.deconvert,temp)

      path.append(temp)
      i = i + 1
      ctr = ctr + 1

    end_time=time.time()
    rospy.loginfo("\nPath Constructed")
    if timing:
      print "Time (s): ", end_time-start_time
    path.reverse()
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
    
    found_solution=False
    q=PriorityQueue()
    cost_so_far={}
    came_from={}
    
    q.put(sind,0)
    #print "sind: ", sind
    #print "eind: ", eind
    came_from[sind]=None
    cost_so_far[sind]=0

    while not q.empty():
      parent=q.get()
      #print "parent: ", parent
      #note: get returns only the index, not the priority
      if parent == eind:
        print "found solution, exiting"
        found_solution=True
        break
      #print parent

      for child in self.children(parent):
        #using is passable1 since we are using precomputed obstacle_infaltion
        if not self.is_passable1(child):
          #print "not passable"
          continue
        
        cost1=cost_so_far[parent]+self.heur(parent,child)
        if child not in cost_so_far or cost1 < cost_so_far[child]:
          #print "in if"
          cost_so_far[child]=cost1
          new_cost=self.heur(child,eind) + cost1
          q.put(child,new_cost)
          #print child
          came_from[child]=parent
    
    if found_solution:
      c=cost_so_far[eind]
    else:
      c=-1

    #print "came from: ", came_from[(334,234)]
    print "leaving astar"
    return [found_solution,came_from,c]

  #no longer using hashing functions, indexing on tuples
  def hasher(self,index):
    return 1000*index[0]+index[1]

  def dehasher(self,hashed_index):
    return [int(math.floor(hashed_index/1000)),hashed_index%1000]
  
  #can be modified to return four or eight children
  def children(self,index):
    return [ (index[0]+1,index[1]), (index[0]-1,index[1]), (index[0],index[1]+1), (index[0],index[1]-1), (index[0]+1,index[1]+1), (index[0]-1,index[1]-1), (index[0]+1,index[1]-1), (index[0]-1,index[1]+1)  ]
  
  
  def is_passable(self,index,dist=3):
    for x in range(index[0]-dist,index[0]+dist):
      for y in range(index[1]-dist,index[1]+dist):
        try:
          if self.map.data[self.indexer(x,y)] == 100:
            return False
        except:
          return False
    return True

  #using precomputed inflated obstacles
  def is_passable1(self,index):
    try:
      #print [x,y]
      if self.map.data[self.indexer(index[0],index[1])] == 100:
        #print "inside"
        return False
    except:
      return False
    #print "returning true"
    return True


  #heuristic #1 - taxicab distance between two points assuming no obstacles
  #represents being able to move in four major directions (no diagonals)
  #might look into changing point formatting in the future
  def manhattan(self,x1,y1,x2,y2):
    return (abs(x1-x2) + abs(y1-y2))
    
  #heuristic #2 - euclidean distance between two points
  #represents being able to move in any direction
  def euclidean(self,x1,y1,x2,y2):
    return (math.sqrt((x1-x2)**2 + (y1-y2)**2))

  #heuristic #3 - diagonal distance
  def diagonal(self,x1,y1,x2,y2):
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return (dx + dy) + (math.sqrt(2)-2)*min(dx,dy)

  #this function is for convenience - change one line here 
  #to alter which hueristic we use in all future functions.
  def heur(self,p1,p2):
    return self.diagonal(p1[0],p1[1],p2[0],p2[1])




