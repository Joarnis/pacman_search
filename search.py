# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
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
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    """
    # Make an node with the given starting position
    node = SearchNode(problem.getStartState())

    # The fringe is managed as a stack
    fringe = util.Stack()
    explored = set()

    fringe.push(node)

    # Graph Search Algorithm Implementation
    while fringe.isEmpty() is False:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node.get_solution()
        if node.state not in explored:
            explored.add(node.state)
            expansion = problem.getSuccessors(node.state)
            for item in expansion:
                # No costs are stored in the new SearchNode (child_node.cost is initialized with 0)
                child_node = SearchNode(item[0], item[1], node)
                fringe.push(child_node)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # Make an node with the given starting position
    node = SearchNode(problem.getStartState())

    # The fringe is managed as a queue
    fringe = util.Queue()
    explored = set()

    fringe.push(node)

    # Graph Search Algorithm Implementation
    while fringe.isEmpty() is False:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node.get_solution()
        if node.state not in explored:
            explored.add(node.state)
            expansion = problem.getSuccessors(node.state)
            for item in expansion:
                # No costs are stored in the new SearchNode (child_node.cost is initialized with 0)
                child_node = SearchNode(item[0], item[1], node)
                fringe.push(child_node)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    # Make an node with the given starting position
    node = SearchNode(problem.getStartState())

    # The fringe is managed as a priority queue
    fringe = util.PriorityQueue()
    explored = set()

    fringe.push(node, 0)

    # Graph Search Algorithm Implementation
    while fringe.isEmpty() is False:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node.get_solution()
        if node.state not in explored:
            explored.add(node.state)
            expansion = problem.getSuccessors(node.state)
            for item in expansion:
                # Costs now are stored in the new SearchNode (child_node.cost is now
                # initialized with the cost to get to the previous node plus the cost from getSuccessors)
                child_node = SearchNode(item[0], item[1], node, item[2] + node.cost)
                fringe.push(child_node, item[2] + node.cost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # Make an node with the given starting position
    node = SearchNode(problem.getStartState())

    # The fringe is managed as a priority queue
    fringe = util.PriorityQueue()
    explored = set()

    fringe.push(node, 0)

    # Graph Search Algorithm Implementation
    while fringe.isEmpty() is False:
        node = fringe.pop()
        if problem.isGoalState(node.state):
            return node.get_solution()
        if node.state not in explored:
            explored.add(node.state)
            expansion = problem.getSuccessors(node.state)
            for item in expansion:
                # Costs now are stored in the new SearchNode (child_node.cost is now
                # initialized with the cost to get to the previous node plus the cost from getSuccessors)
                child_node = SearchNode(item[0], item[1], node, item[2] + node.cost)
                # The child node is pushed into the fringe with a priority of the cost to get
                # to that node plus the result of the heuristic function for that node
                fringe.push(child_node, item[2] + node.cost + heuristic(item[0], problem))

class SearchNode:
    """
    Custom node used in search algorithms (DFS, BFS, UCS, A*)
    """

    def __init__(self, state, action = None, parent = None, cost = 0):
        self.state = state
        self.action = action
        self.parent = parent
        self.cost = cost

    # Recursively get the actions that led to this state (Node)
    def get_solution(self):
        solution = []

        if self.parent is not None:
            solution = self.parent.get_solution()
            solution.append(self.action)

        return solution


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
