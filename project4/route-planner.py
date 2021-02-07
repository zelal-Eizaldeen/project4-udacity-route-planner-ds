import math   
# from helpers import Map, load_map_10, load_map_40, show_map
class Map():
  def __init__(self, intersections, roads):
    self.intersections = intersections
    self.roads = roads
class Path_Planner():
  def __init__(self, Map, start=None, goal=None):
    self.map = Map
    self.start = start
    self.goal = goal
    self.closedSet = self.create_closedSet() if goal != None and start != None else None
    self.openSet = self.create_openSet() if goal != None and start != None else None
    self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
    self.gScore = self.create_gScore() if goal != None and start != None else None
    self.fScore = self.create_fScore() if goal != None and start != None else None
    self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    self.intersections = intersections
    self.roads = roads
  def reconstruct_path(self, current):
    total_path = [current]
    while current in self.cameFrom.keys():
        current = self.cameFrom[current]
        total_path.append(current)
    return total_path
  def create_closedSet(self):
    """ Creates and returns a data structure suitable to hold the set of nodes already evaluated"""
    return set()
  def create_openSet(self):
    """ Creates and returns a data structure suitable to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    if self.start != None:
      openSet = set()
      openSet.add(self.start)
      return openSet
    
    raise(ValueError, "Must create start node before creating an open set.")
  def create_cameFrom(self):
    """Creates and returns a data structure that shows which node can most efficiently be reached from another,
    for each node."""
    cameFrom = {}
    return cameFrom
  def create_gScore(self):
    """Creates and returns a data structure that holds the cost of getting from the start node to that node, 
    for each node. The cost of going from start to start is zero."""
    if self.start != None:
      gScore = {}
      for node in self.map.intersections.keys():
        if node == self.start:
          gScore[node] = 0
        else: gScore[node] = float('inf')
      return gScore
    raise(ValueError, "Must create start node before creating gScore. Try running PathPlanner.set_start(start_node)")
  def create_fScore(self):
    """Creates and returns a data structure that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    if self.start != None:
        fScore = {}
        for node in self.map.intersections.keys():
            if node == self.start:
                fScore[node] = self.heuristic_cost_estimate(self.start)
            else: fScore[node] = float('inf')
        return fScore
    raise(ValueError, "Must create start node before creating fScore. Try running PathPlanner.set_start(start_node)")
  def _reset(self):
    """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
    self.closedSet = None
    self.openSet = None
    self.cameFrom = None
    self.gScore = None
    self.fScore = None
    self.path = self.run_search() if self.map and self.start and self.goal else None
  
  def run_search(self):
    if self.map == None:
        raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
    if self.goal == None:
        raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
    if self.start == None:
        raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

    self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
    self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
    self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
    self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
    self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

    while not self.is_open_empty():
      current = self.get_current_node()
      if current == self.goal:
        self.path = [x for x in reversed(self.reconstruct_path(current))]
        return self.path
      else:
          self.openSet.remove(current)
          self.closedSet.add(current)

      for neighbor in self.get_neighbors(current):
          if neighbor in self.closedSet:
              continue    # Ignore the neighbor which is already evaluated.

          if not neighbor in self.openSet:    # Discover a new node
              self.openSet.add(neighbor)
          
          # The distance from start to a neighbor
          #the "dist_between" function may vary as per the solution requirements.
          if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
              continue        # This is not a better path.

          # This path is the best until now. Record it!
          self.record_best_path_to(current, neighbor)
    print("No Path Found")
    self.path = None
    return False
  

  def set_map(self, Map):
    self._reset(self)
    self.start = None
    self.goal = None
    self.map = Map
  def set_start(self, start):
    self._reset(self)
    self.start = start
    self.goal = None
  def set_goal(self, goal):
    self._reset(self)
    self.goal = goal
  def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    return not bool(self.openSet)
  def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    current = None
    minim = float('inf')
    for node in self.openSet:
        if self.fScore[node] < minim:
            minim = self.fScore[node]
            current = node
    return current
  def get_neighbors(self, node):
    return set(self.map.roads[node])
  def get_gScore(self, node):
    return self.gScore[node]
  def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    x1 = self.map.intersections[node_1][0]
    y1 = self.map.intersections[node_1][1]
    x2 = self.map.intersections[node_2][0]
    y2 = self.map.intersections[node_2][1]
    dist = math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
    return dist
  def get_tentative_gScore(self, current, neighbor):
    g_score_current = self.get_gScore(current)
    dist_current_neighbor = self.distance(current,neighbor)
    return g_score_current+dist_current_neighbor
  def heuristic_cost_estimate(self, node):
    if self.goal != None:
        heuristic_estimate = self.distance(node,self.goal)
        return heuristic_estimate
    raise(ValueError, "Must create goal node before calculating huristic estimate. Try running PathPlanner.set_goal(goal_node)")
  def calculate_fscore(self, node):
    f_score = self.get_gScore(node) + self.heuristic_cost_estimate(node)
    return f_score
  def record_best_path_to(self, current, neighbor):
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tentative_gScore(current,neighbor)
    self.fScore[neighbor] = self.gScore[neighbor] + self.heuristic_cost_estimate(neighbor)
  
intersections = {
        0: [0.7798606835438107, 0.6922727646627362],
        1: [0.7647837074641568, 0.3252670836724646],
        2: [0.7155217893995438, 0.20026498027300055],
        3: [0.7076566826610747, 0.3278339270610988],
        4: [0.8325506249953353, 0.02310946309985762],
        5: [0.49016747075266875, 0.5464878695400415],
        6: [0.8820353070895344, 0.6791919587749445],
        7: [0.46247219371675075, 0.6258061621642713],
        8: [0.11622158839385677, 0.11236327488812581],
        9: [0.1285377678230034, 0.3285840695698353]}
roads = [[5,6,7], [2,3,4],[4, 3, 1],[5, 4, 1, 2],[1, 2, 3],[7, 0, 3],[0],[0, 5],[9],[8]]
map_10 = Map(intersections, roads)

planner = Path_Planner(map_10, 5, 6)
path = planner.path
if path == [5, 0, 6]:

# if path == [5, 16, 37, 12, 34]:
    print("great! Your code works for these inputs!")
else:
    print("something is off, your code produced the following:")
    print(path)

