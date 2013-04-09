
# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and Pieter 
# Abbeel in Spring 2013.
# For more info, see http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]
   

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first [p 74].
  
    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
    """
    "*** YOUR CODE HERE ***"
    # import Stack and Directions
    from game import Directions 
    from util import Stack
    from util import Queue
  
    # create closed set, fringe, and path
    closed = {}
    fringe = Stack()
    path = []

    # get root node and add to fringe
    root = problem.getStartState()
    fringe.push((root, root, 'Stop'))
  
    # while nodes still exist on fringe
    while not fringe.isEmpty():
        node = fringe.pop()

        if problem.isGoalState(node[0]):
            while node[2] != 'Stop':
                path.append(node[2])
                node = closed[node[1]]
            path.reverse()
            return path

        closed[node[0]] = node

        children = problem.getSuccessors(node[0])

        for child in children:
            if child[0] not in closed:
                fringe.push((child[0], node[0], child[1]))

    return None


def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"
  from util import Queue
  
  #util.raiseNotDefined()
  currentState = problem.getStartState()
  
  setOfExploredNodes = set([currentState])
  listofMoves = []
  successorStack = Queue()
  counter = 0;
  
  while not problem.isGoalState(currentState):
      for successor in problem.getSuccessors(currentState):          
          if successor[0] not in setOfExploredNodes:
              #print successor
              setOfExploredNodes.add(successor[0])
              successorStack.push((successor[0], listofMoves + [successor[1]]))              
      
      currentState = successorStack.pop()
      
      #setOfExploredNodes.add(currentState[0])
      listofMoves = currentState[1]
      currentState = currentState[0]
      
      counter += 1
      if counter % 100 == 0:
        print counter
        

  print listofMoves  
  return listofMoves
      

def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    return aStarSearch(problem, nullHeuristic)


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0


def aStarSearch(problem, heuristic = nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    
    
    from game import Directions 
    from util import PriorityQueue
    
    
    closed = {}
    fringe = PriorityQueue()
    path = []

    
    g = {}
    h = {}
    f = {} 
    
   
    root = problem.getStartState()
    
    
    g[root] = 0
    h[root] = heuristic(root, problem)
    f[root] = g[root] + h[root]

    
    fringe.push((root, None, 'Stop'), f[root])

    
    while not fringe.isEmpty():
       
        node = fringe.pop()
        #print node
        
        if problem.isGoalState(node[0]):
            
            while node[2] != 'Stop':
                path.append(node[2])
                node = closed[node[1]]
    
            path.reverse()
            return path
    
        
        closed[node[0]] = node

        
        children = problem.getSuccessors(node[0])
        for child in children:
            
            if child[0] not in closed:
             
                tg = g[node[0]] + child[2]
                better = False
                
              
                if not h.has_key(child[0]):
                    childNode = (child[0], node[0], child[1])
                    h[child[0]] = heuristic(child[0], problem)
                    better = True
              
                elif tg < g[child[0]]:
                    better = True
                
               
                if better:
                    g[child[0]] = tg
                    f[child[0]] = g[child[0]] + h[child[0]]
                    fringe.push(childNode, f[child[0]])
                    
    
    return None
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
  
