#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
import sys
from search import * #for search engines
from search import _BEST_FIRST, _CUSTOM
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

from collections import deque
import hungarian as h


# states by boxes
UNVISITED=0
VISITED=1
BAD=2
GOOD=3
SKETCH=4

bs_dist_cache = None
box_cache = None

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    return find_total_dist(state.boxes, state.storage)

# Manhattan
def dist(b, s):
    return sum( [ abs(s[i]-b[i]) for i in [0,1] ] )

def find_total_dist(boxes, storage):
    total = 0
    if not boxes: return 0
    if not storage: return sys.max_value
    for b in boxes:
        m = None
        for s in storage:
            if not m or dist(b,s) < m:
                m = dist(b,s)
        total += m
    return total

def find_dist_util_with_replacement(boxes, storages):
    # backtracking to find minimum total distances
    if not boxes: return 0
    b = list(boxes)
    s = list(storages)
    min_d = None
    for i,_ in enumerate(boxes):
        n_boxes = b[:i]+b[i+1:]
        for j,_ in enumerate(storages):
            n_storages = s[:j]+s[j+1:]
            d = dist(b[i],s[j]) + find_dist_util_with_replacement(n_boxes, n_storages)
            if not min_d or min_d > d: min_d = d
    return min_d

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    global box_cache
    global bs_dist_cache
    if not box_cache or state.action == "START":
        box_cache = state.boxes
    if box_cache == state.boxes and bs_dist_cache:
        if bs_dist_cache == sys.maxsize: return bs_dist_cache
        return bs_dist_cache + min_rb_not_s_dist(state)

    box_cache = state.boxes

    # if boxes are cornered, state is not valid 
    if not valid(state): return sys.maxsize

    # use hungarian algorithm to find min distance of box to storage WHERE NO TWO BOX SHARES SAME STORAGE
    D = h.build_dist_table(state.boxes, state.storage)
    bs = h.Hungarian(D)
    bs_dist, bs_pairs = bs.compute()
    bs_dist_cache = bs_dist

    return bs_dist + min_rb_not_s_dist(state)


def min_rb_not_s_dist(state):
    not_stored_boxes = state.boxes.difference(state.storage)
    min_dist = None
    for r in state.robots:
        for b in not_stored_boxes:
            if not min_dist or dist(r,b) < min_dist:
                min_dist = dist(r,b)
    if not min_dist: return 0
    return min_dist

# A DFS flood-fill algorithm to search if box is blocked
def valid(state):
    visiting = {}
    blocked = {}
    for box in state.boxes:
        visiting[box] = False
        blocked[box] = UNVISITED

    for box in state.boxes:
        if is_blocked(state, box, visiting, blocked)==BAD: return False
    for box in state.boxes:
        if blocked[box] == SKETCH: 
            if not reachable(state,box): 
                return False
    return True

# DFS approach to find robot
def reachable(state,box):
    n = get_neighbors(state,box)
    new_obstacles = set(state.obstacles)
    for l in n:
        if is_box(state,l): 
            new_obstacles.add(l)

    visited = set()
    #bfs loop based
    dq =deque()
    dq.append(box)
    while dq:
        node = dq.popleft()
        if node in state.robots: return True
        for nei in get_neighbors(state,node):
            if nei not in visited and nei not in new_obstacles:
                dq.append(nei)
            visited.add(node)
    return False


def get_neighbors(state,box):
    neighbors = []
    if box[1]-1 >= 0:
        neighbors.append( (box[0],box[1]-1) )
    if box[1]+1 < state.height:
        neighbors.append( (box[0],box[1]+1) )
    if box[0]+1 < state.width:
        neighbors.append( (box[0]+1,box[1]) )
    if box[0]-1 >= 0:
        neighbors.append( (box[0]-1,box[1]) )
    return neighbors

def is_valid_location(state,location):
    if location[0] < 0 or location[0] >= state.width: return False
    if location[1] < 0 or location[1] >= state.height: return False
    for obstacles in state.obstacles:
        if location == obstacles: return False
    return True

# DP approach to if box is blocked
def is_blocked(state,box,visiting,blocked):

    if is_storage(state,box): return GOOD
    if blocked[box] != UNVISITED: return blocked[box]
    if visiting[box]: 
        # deadlock
        blocked[box] = BAD
        return blocked[box]

    n = (box[0],box[1]-1)
    s = (box[0],box[1]+1)
    e = (box[0]+1,box[1])
    w = (box[0]-1,box[1])

    visiting[box] = True

    # depth first backtracking search
    for loc1,loc2 in [ (n,e), (e,s), (s,w), (w,n) ]:
        #if is_valid_location(state,loc1) or is_valid_location(state,loc2): continue

        if not is_box(state,loc1) and not is_box(state,loc2): 
            if not is_valid_location(state,loc1) and not is_valid_location(state,loc2): 
                blocked[box]=BAD
                return blocked[box]

        elif is_box(state,loc1) and not is_box(state,loc2):
            if not is_valid_location(state,loc2):
                t = is_blocked(state,loc1,visiting,blocked)
                if t == VISITED: blocked[loc1] = SKETCH # make sure loc1 is reachable by robot
                if t == BAD:
                    blocked[box]=BAD
                    return blocked[box]

        elif is_box(state,loc2) and not is_box(state,loc1):
            if not is_valid_location(state,loc1):
                t = is_blocked(state,loc2,visiting,blocked)
                if t == VISITED: blocked[loc2] = SKETCH
                elif t == BAD:
                    blocked[box]=BAD
                    return blocked[box]
        # both are boxes
        else: 
            t1 = is_blocked(state, loc1, visiting,blocked)
            t2 = is_blocked(state,loc2,visiting,blocked)
            if t1 == BAD and t2 == BAD:
                blocked[box]=BAD
                return blocked[box]

    # backtrack
    visiting[box] = False
    # 1 is not blocked
    blocked[box]=VISITED
    return blocked[box]

def is_box(state,location):
    for box in state.boxes:
        if box == location: return True
    return False

def is_storage(state,box):
    for storage in state.storage:
        if box == storage: return True
    return False

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  w = 20
  wrapped_fval_function = ( lambda sN: fval_function(sN, w) )

  frontier = Open(_CUSTOM)

  node = sNode(initial_state, heur_fn(initial_state), wrapped_fval_function)

  stop_time = os.times()[0] + timebound

  # for cycle checking
  ht = dict()
  ht[initial_state.hashable_state()] = initial_state.gval

  frontier.insert(node)
  best_node, best_fval = None, None

  while not frontier.empty():
      node = frontier.extract()

      if sokoban_goal_state(node.state):
          # update best_fval
          if not best_fval or node.fval_function(node) < best_fval:
              best_fval = node.fval_function(node)
              best_node = node

      # prune this state if hval == infinity
      if node.hval == sys.maxsize: continue

      if os.times()[0] > stop_time: 
          if not best_node: 
              print("TRACE: Search has exceeeded the time bound provided.")
              print("Final weight: ", w)
              return None
          print("Times up. Returning best node")
          print("Final weight: ", w)
          return best_node.state
            

      if ht[node.state.hashable_state()] < node.gval: continue
      successors = node.state.successors()

      # decrease weight 
      if w-0.0001 > 1: w -= 0.0001
      wrapped_fval_function = (lambda sN: fval_function(sN, w))

      for succ in successors:
            
          hash_state = succ.hashable_state()

          hval = heur_fn(succ)

          succ_node = sNode(succ, hval, wrapped_fval_function)
          # prune if gval > best_gval
          if best_fval and succ_node.fval_function(succ_node) > best_fval: continue
          # 
          if hash_state in ht and succ.gval > ht[hash_state]: continue
          # path checking
          if succ.has_path_cycle(): continue

          ht[hash_state] = succ.gval

          # pass heur_fn as input
          frontier.insert(succ_node)
          
  if not best_node: return None
  return best_node.state

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  frontier = Open(_BEST_FIRST)
  node = sNode(initial_state, heur_fn(initial_state), None)      

  stop_time = os.times()[0] + timebound

  # for cycle checking
  ht = dict()
  ht[initial_state.hashable_state()] = initial_state.gval

  frontier.insert(node)
  best_node, best_gval = None, None

  while not frontier.empty():
      node = frontier.extract()

      if sokoban_goal_state(node.state):
          # update best_gval
          if not best_gval or node.gval < best_gval:
              best_gval = node.gval
              best_node = node

      # prune this state if hval == infinity
      if node.hval == sys.maxsize: continue

      if os.times()[0] > stop_time: 
          if not best_node: 
              #print("TRACE: Search has exceeeded the time bound provided.")
              return None
          #print("Times up. Returning best node")
          return best_node.state

      if ht[node.state.hashable_state()] < node.gval: continue
      successors = node.state.successors()

      for succ in successors:
            
          hash_state = succ.hashable_state()
          # prune if gval > best_gval
          if best_gval and succ.gval > best_gval: continue
          # 
          if hash_state in ht and succ.gval > ht[hash_state]: continue
          # path checking
          if succ.has_path_cycle(): continue

          ht[hash_state] = succ.gval

          hval = heur_fn(succ)
          # pass heur_fn as input
          frontier.insert(sNode(succ, hval, None))
    
  if not best_node: return None
  return best_node.state
