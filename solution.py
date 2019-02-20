#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
import copy
import itertools
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems
len_benchmark_gbfs = [18, 4, 21, 12, 9, -99, 18, 41, 14, -99, 39, 38, -99, 34, 29, -99, -99, -99, -99, -99]
len_benchmark_astar = [18, 4, 21, 10, 8, -99, 16, 41, 16, -99, 39, 38, -99, 35, 29, -99, -99, -99, -99, -99]

##################################globals:######################################
INF = 100000000000

##################################helpers:######################################
#################################goal_state:####################################
def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True
  
#################################fval_function:#################################
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
  return sN.gval + weight*sN.hval
  
###################################doomedBox:###################################
def doomedBox(box, storage, height, width):
  if box[0] == 0 and storage[0] != 0:
    return True
  elif box[1] == 0 and storage[1] != 0:
    return True
  elif box[0] + 1 == width and storage[0] + 1 != width:
    return True
  elif box[1] + 1 == height and storage[1] + 1 != height:
    return True
  return False
  
#################################clusterCheck:##################################
def clusterCheck(state):
  boxes = copy.deepcopy(list(state.boxes))
  storages = copy.deepcopy(list(state.storage))
  obstacles = copy.deepcopy(list(state.obstacles))
  height = state.height
  width = state.width
  top, bottom, left, right = False, False, False, False
  topObs, bottomObs, leftObs, rightObs = False, False, False, False
  for box in boxes:
    if box in storages:
      continue
    if box[0] == 0 or box[0] == width - 1:
      if (box[0], box[1]+1) in (boxes + obstacles):
        return INF
      elif (box[0], box[1]-1) in (boxes + obstacles):
        return INF
    if box[1] == 0 or box[1] == height - 1:
      if (box[0]+1, box[1]) in (boxes + obstacles):
        return INF
      elif (box[0]-1, box[1]) in (boxes + obstacles):
        return INF
    
    
    if box[0] - 1 in obstacles:
      leftObs = True
    if box[0] + 1 in obstacles:
      rightObs = True
    if box[1] - 1 in obstacles:
      bottomObs = True
    if box[1] + 1 in obstacles:
      topObs = True
        
    if (topObs + bottomObs + leftObs + rightObs) >= 3:
      return INF
    
  return 0

###############################storageMomentum:###################################  
def thePhantomFunction(state):
  phantomDecrement = -2
  if state.parent == None:
    return 0
    
  storages = copy.deepcopy(list(state.storage))
  
  curBoxes = copy.deepcopy(list(state.boxes))
  curBoxes = set(curBoxes)

  prevBoxes = copy.deepcopy(list(state.parent.boxes))
  prevBoxes = set(prevBoxes)
  
  prevBox = prevBoxes - curBoxes
  prevBox = list(prevBox)
  curBox = curBoxes - prevBoxes
  curBox = list(curBox)
  
  if curBox == []:
    return 0
  
  curBoxes = copy.deepcopy(list(state.boxes))
  for box in curBoxes:
    if box in storages:
      return phantomDecrement
  return 0
  
###############################occupiedStorages:#################################
def occuppiedStorages(state):
  total = 0
  robots = copy.deepcopy(list(state.robots))
  storages = copy.deepcopy(list(state.storage))
  for robot in robots:
    if robot in storages:
      total += 1
  return total
################################computeManhattan:###############################
def computeManhattan(a, b):
  return abs(a[0] - b[0]) + abs(a[1] - b[1])
  
def minkowskiDistance(a, b):
  n = 4
  aComponent = abs(a[0] - b[0])
  aComponent = aComponent**n
  bComponent = abs(a[1] - b[1])
  bComponent = bComponent**n
  return (aComponent + bComponent)**1/n
##########################manhattan approach:#################################
def manhattanMatrix(state, primaryObjects, secondaryObjects):
  obstacles = copy.deepcopy(list(state.obstacles))
  matrix = [ [ 0 for i in range(len(secondaryObjects))] for j in range(len(primaryObjects))]
  for i in range(len(primaryObjects)):
    for j in range(len(secondaryObjects)):
      if secondaryObjects == list(state.storage):
        if doomedBox(primaryObjects[i], secondaryObjects[j], state.height, state.width):
          matrix[i][j] = INF
        else:
          matrix[i][j] = computeManhattan(primaryObjects[i], secondaryObjects[j])
      else:
        matrix[i][j] = computeManhattan(primaryObjects[i], secondaryObjects[j])
  return matrix
  
def boxStorageHeurManhattan(box, storage, height, width):
  if doomedBox(box, storage, height, width):
    return INF
  return computeManhattan(box, storage)
  
def boxRobotHeurManhattan(box, robot):
  return computeManhattan(box, robot)
  
############################minkowski approach:#################################
def minkowskiMatrix(state, primaryObjects, secondaryObjects):
  obstacles = copy.deepcopy(list(state.obstacles))
  matrix = [ [ 0 for i in range(len(secondaryObjects))] for j in range(len(primaryObjects))]
  for i in range(len(primaryObjects)):
    for j in range(len(secondaryObjects)):
      if secondaryObjects == list(state.storage):
        if doomedBox(primaryObjects[i], secondaryObjects[j], state.height, state.width):
          matrix[i][j] = INF
        else:
          matrix[i][j] = minkowskiDistance(primaryObjects[i], secondaryObjects[j])
      else:
        matrix[i][j] = minkowskiDistance(primaryObjects[i], secondaryObjects[j])
  return matrix

#############################matching problem:##################################
###########################permutation approach:################################
def computeMatchingProblem(matchingMatrix):
  permutations = list(itertools.permutations([ i for i in range(len(matchingMatrix)) ]))
  permutationSums = []
  for i in range(len(permutations)):
    callingMatrix = copy.deepcopy(matchingMatrix)
    for j in range(len(permutations[i])):
      callingMatrix[j] = copy.deepcopy(matchingMatrix[permutations[i][j]])
    permutationSums.append(findPermutationSum(callingMatrix, 0))
  return min(permutationSums)
  
def findPermutationSum(matchingMatrix, sum):
  rowCount = len(matchingMatrix)
  if rowCount ==  1:
    sum += min(matchingMatrix[0])
    return sum
  minimum = min(matchingMatrix[0])
  minIndex = matchingMatrix[0].index(minimum)
  sum += minimum
  for i in range(1, len(matchingMatrix)):
    matchingMatrix[i][minIndex] = INF
  del(matchingMatrix[0])
  return findPermutationSum(matchingMatrix, sum)

#############################min of all rows approach:##########################
def minRowSum(matchingMatrix):
  sum = 0
  for i in range(len(matchingMatrix)):
    min = INF
    for j in range(len(matchingMatrix[0])):
      if matchingMatrix[i][j] < min:
        min = matchingMatrix[i][j]
    sum += min
  return sum
  
##########################manhattan with permutations:##########################
def manhattanWithPermutations(state):
  boxes = copy.deepcopy(list(state.boxes))
  storages = copy.deepcopy(list(state.storage))
  robots = copy.deepcopy(list(state.robots))
  
  boxesAndStorageMatrix = manhattanMatrix(state, boxes, storages)
  boxesAndRobotsMatrix = manhattanMatrix(state, robots, boxes)
  
  boxesAndStorageHeur = computeMatchingProblem(boxesAndStorageMatrix)
  boxesAndRobotsHeur = computeMatchingProblem(boxesAndRobotsMatrix)
  
  return boxesAndStorageHeur + boxesAndRobotsHeur
  
###########################minkowski with permutations:#########################
def minkowskiWithPermutations(state):
  boxes = copy.deepcopy(list(state.boxes))
  storages = copy.deepcopy(list(state.storage))
  robots = copy.deepcopy(list(state.robots))
  
  boxesAndStorageMatrix = minkowskiMatrix(state, boxes, storages)
  boxesAndRobotsMatrix = minkowskiMatrix(state, robots, boxes)
  
  boxesAndStorageHeur = minRowSum(boxesAndStorageMatrix)
  boxesAndRobotsHeur = minRowSum(boxesAndRobotsMatrix)
  
  return boxesAndStorageHeur + boxesAndRobotsHeur
  
#########################assignment heuristic functions:########################
###############################heur_alternate###################################
def heur_alternate(state):
  #IMPLEMENT
  '''a better heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  #heur_manhattan_distance has flaws.
  #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
  #Your function should return a numeric value for the estimate of the distance to the goal. 
  if state.height <= 5 or state.width <= 5:
    return manhattanWithPermutations(state) + clusterCheck(state) + occuppiedStorages(state)
  else:
    return minkowskiWithPermutations(state) + clusterCheck(state) + occuppiedStorages(state) + thePhantomFunction(state)

################################heur_zero#######################################
def heur_zero(state):
  '''Zero Heuristic can be used to make A* search perform uniform cost search'''
  return 0

############################heur_manhattan_distance#############################
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
  
  # Extracting sets from frozen sets:
  boxes = list(state.boxes)
  storages = list(state.storage)
  
  # Initializing Manhattan distance:
  totalManhattan = 0
  for box in boxes:
    # Setting min distance to be larger than h*
    min = 2 * (state.height + state.width)
    for storage in storages:
      # Looping for closest storage:
      manhattan = computeManhattan(box, storage)
      if manhattan < min:
        min = manhattan
    totalManhattan += min
  
  return totalManhattan


################################trivial_heuristic###############################
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

#############################search strategies##################################
#########################anytime_weighted_astar#################################
def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''  
  #################################init:########################################
  start_time = os.times()[0]
  se = SearchEngine('custom', 'full')
  se.trace_off()
  wrappedfVal = (lambda sN : fval_function(sN,weight))
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrappedfVal)
  
  ################################first run:####################################
  timeRemaining = timebound - (os.times()[0] - start_time)
  finalNode = se.search(timebound = timeRemaining)
    
  ################################improvements:#################################
  costTriple = [INF, INF, INF]
  if finalNode:
    costTriple[2] = finalNode.gval
  while timeRemaining > 0 and weight >= 1:
    currentNode = se.search(timebound = timeRemaining, costbound = costTriple)
    if currentNode == False:
      break
    if finalNode != False:
      costTriple[2] = finalNode.gval
    finalNode = currentNode
    weight -= 1
    timeRemaining = timebound - (os.times()[0] - start_time)
  return finalNode
  
##################################anytime_gbfs##################################
def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  #################################init:########################################
  start_time = os.times()[0]
  se = SearchEngine('best_first', 'full')
  se.trace_off()
  se.init_search(initial_state, sokoban_goal_state, heur_fn, fval_function)
  
  ################################first run:####################################
  timeRemaining = timebound - (os.times()[0] - start_time)
  finalNode = se.search(timebound = timeRemaining)
  if finalNode == False:
    return False
    
  ################################improvements:#################################
  costTriple = [INF, INF, INF]
  costTriple[0] = finalNode.gval
  while timeRemaining > 0:
    currentNode = se.search(timeRemaining, costTriple)
    if currentNode == False:
      break
    if currentNode.gval < finalNode.gval:
      finalNode = currentNode
      costTriple[0] = finalNode.gval
    timeRemaining = timebound - (os.times()[0] - start_time)
  return finalNode


if __name__ == "__main__":
  anytime_gbfs(PROBLEMS[0], heur_alternate, timebound = 10)
  weight = 10
  anytime_weighted_astar(PROBLEMS[0], heur_alternate, weight, timebound = 10)
  
  


  
  

  
  
  
  
  

  
  
  
    

