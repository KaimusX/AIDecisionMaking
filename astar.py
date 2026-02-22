# Credit for this: Nicholas Swift
# as found at https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
from warnings import warn
import heapq, time

class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement = False):
    """
    Returns a list of tuples as a path from the given start to the given end in the given maze.
    Also prints the total path cost, number of nodes generated including duplicates,and elapssed runtime.
    :param maze:
    :param start:
    :param end:
    :return:
    """
    # timing start
    t0 = time.perf_counter()

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # heap and count of nodes generated
    heapq.heapify(open_list)
    heapq.heappush(open_list, start_node)
    node_count = 1   # start node counts as created

    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 2)

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1

        if outer_iterations > max_iterations:
          # if we hit this point return the path such as it is
          # it will not contain the destination
          warn("giving up on pathfinding too many iterations")
          return return_path(current_node)       
        
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = return_path(current_node)
            cost = sum(maze[pos[0]][pos[1]] for pos in path[1:]) # 1: excludes start node
            elapsed = (time.perf_counter() - t0) * 1000.0  # ms
            print(f"Path found: {path}\nCost: {cost}\nNodes generated: {node_count}\nTime: {elapsed:.3f}ms")
            return path

        # Generate children
        children = []
        
        for new_position in adjacent_squares: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] == 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)
            node_count += 1  # count every instantiation regardless of duplication

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + maze[child.position[0]][child.position[1]]
            # squared euclidian
            #  child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            heapq.heappush(open_list, child)

    warn("Couldn't get a path to destination")
    path = return_path(current_node)
    cost = sum(maze[pos[0]][pos[1]] for pos in path[1:])
    elapsed = (time.perf_counter() - t0) * 1000.0
    print(f"Partial path: {path}\nCost: {cost}\nNodes generated: {node_count}\nTime: {elapsed:.3f}ms")
    return path

def example(print_maze = True):

    maze = [[1,4,3,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [1,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [1,1,1,5,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,] * 2,
            [1,0,0,5,1,0,0,1,1,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,] * 2,
            [1,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,] * 2,
            [0,0,0,1,0,1,1,1,1,0,1,1,0,0,1,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,] * 2,
            [0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,1,1,0,1,0,0,0,0,0,0,1,1,1,0,] * 2,
            [0,0,0,1,0,1,1,0,1,1,0,1,1,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0,0,0,] * 2,
            [0,0,0,1,0,1,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,1,0,1,0,1,1,] * 2,
            [0,0,0,1,1,1,0,1,1,0,1,1,1,1,0,0,1,1,1,1,1,1,1,0,1,0,1,0,0,0,] * 2,
            [5,5,5,1,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,1,1,0,] * 2,
            [1,0,0,1,0,1,1,1,1,0,1,0,0,1,1,1,0,1,1,1,1,0,1,1,1,0,1,0,0,0,] * 2,
            [1,0,0,1,1,1,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,1,] * 2,
            [1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,1,] * 2,
            [1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,] * 2,]
    
    start = (0, 0)
    end = (len(maze)-3, len(maze[0])-60)

    # TODO: Add switch to select maze.
    """
    MAZE 1
     Start 1,2
     Goal 4,3
    """
    maze1 = [[2,4,2,1,4,5,2],
             [0,1,2,3,5,3,1],
             [2,0,4,4,1,2,4],
             [2,5,5,3,2,0,1],
             [4,3,3,2,1,0,1],
            ]
    
    """
    MAZE 2
     Start 3,6
     Goal 5,1
    """
    maze2 = [[1,3,2,5,1,4,3],
             [2,1,3,1,3,2,5],
             [3,0,5,0,1,2,2],
             [5,3,2,1,5,0,3],
             [2,4,1,0,0,2,0],
             [4,0,2,1,5,3,4],
             [1,5,1,0,2,4,1],
             ]
    
    """
    MAZE 3
     Start 1,2
     Goal 8,8
    """
    maze3 = [[2,0,2,0,2,0,0,2,2,0],
             [1,2,3,5,2,1,2,5,1,2],
             [2,0,2,2,1,2,1,2,4,2],
             [2,0,1,0,1,1,1,0,0,1],
             [1,1,0,0,5,0,3,2,2,2],
             [2,2,2,2,1,0,1,2,1,0],
             [1,0,2,1,3,1,4,3,0,1],
             [2,0,5,1,5,2,1,2,4,1],
             [1,2,2,2,0,2,0,1,1,0],
             [5,1,2,1,1,1,2,0,1,2],
             ]
    # TODO: add two more mazes of our own design minimum 10x10
    

    path = astar(maze, start, end)

    # TODO: Update print logic
    if print_maze:
      for step in path:
        maze[step[0]][step[1]] = 9 # actual step in route
      
      for row in maze:
        line = []
        for col in row:
          if col == 0:
            line.append("\u2588")
          elif col in range(1, 6):
            line.append(" ")
          elif col == 9:
            line.append(".")
        print("".join(line))

def main():
   example()

# TODO Update parameters to be runnale from cmd line
if __name__ == '__main__':
   main()