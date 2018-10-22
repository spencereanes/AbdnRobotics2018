import heapq

class PriorityQueue:
  def __init__(self):
    self.items=[]

  def empty(self):
    return len(self.items)==0

  def put(self,item,priority):
    heapq.heappush(self.items, (priority, item))

  def get(self):
    return heapq.heappop(self.items)[1]
