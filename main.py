'''main.py
   
   Appendix C: 
   Main script to run the A*/D* algorithms on a Manhattan grid graph.
'''

from manhattangrid import ManhattanGrid
from astar import AStar
from dstar import *
from utils import Color

# Define colors
RED = Color.RED
DARK_RED = Color.DARK_RED
BARK = Color.BARK
GREEN = Color.GREEN
SKY = Color.SKY
PURPLE = Color.PURPLE

# algorithm lists
ASTAR = ["A*", "a*", "A", "a"]
DSTAR = ["D*", "d*", "D", "d"]


def init_env():
    '''Initialize the simulation environment.'''
    # Initialize the graph
    graph = ManhattanGrid(-73.9709,40.7782,-73.9475,40.801)  # bounding box for 97th transverse
    print("Manhattan, NY map initialized.")

    # user choose which algorithm to run
    algorithm = input("Enter the algorithm you want to use (A* or D*): ")
    while algorithm not in ASTAR and algorithm not in DSTAR:
        algorithm = input("Invalid algorithm. Enter the algorithm you want to use (A* or D*): ")


    # hardcoded start and goal intersections and algorithm
    start = graph.mgnodes[42448726] # East 92nd Street and 5th Avenue
    goal = graph.mgnodes[42421728] # West 106th Street and Central Park West
    graph.set_start(start)
    graph.set_goal(goal)
    graph.show(wait="Hit return")

    return graph, algorithm

def on_key(event, graph: ManhattanGrid):
    '''Handle key events for graph movement.'''
    if not hasattr(on_key, 'km'):
        on_key.km = 0
    if not hasattr(on_key, 'U'):
        on_key.U = []
    if not hasattr(on_key, 's_curr'):
        on_key.s_curr = graph.start
    if not hasattr(on_key, 'path'):
        on_key.path = [on_key.s_curr]

    on_key.U, on_key.km = init_dstar(graph, on_key.U, on_key.km)

    if event.key == 'b':
        graph.block_edge()
        graph.update_visualization(on_key.path)
    elif event.key == ' ':
        if on_key.s_curr == graph.goal:
            print("Goal reached")
            graph.plot_path(on_key.path)
            return
        s_new, on_key.km = replan(graph, on_key.U, on_key.s_curr, on_key.km)
        if s_new is None:
            print("No known path to goal")
            return
        on_key.path.append(s_new)
        on_key.s_curr.curr = False
        on_key.s_curr = s_new
        on_key.s_curr.curr = True
        graph.update_visualization(on_key.path)

    

def main():
    graph, algorithm = init_env()
    start = graph.start
    goal = graph.goal
    # Create a function to show each step
    def show(wait=0.0000025, update_interval=3):
        if hasattr(show, 'counter'):
            show.counter += 1
        else:
            show.counter = 0
        if show.counter % update_interval == 0:
            for mg in graph.mgnodes.values():
                if mg.done:
                    graph.color(mg.x, mg.y, BARK)
                elif mg.seen:
                    graph.color(mg.x, mg.y, GREEN)
                elif mg == start:
                    graph.color(mg.x, mg.y, DARK_RED)
                elif mg == goal:
                    graph.color(mg.x, mg.y, DARK_RED)
                else:
                    graph.color(mg.x, mg.y, SKY)
            graph.show(wait)

    if algorithm in ASTAR:
        # Initialize AStar class
        astar = AStar(graph)
        # Run A* algorithm
        path = astar.find_path(show)
        # Plot the path
        graph.plot_path(path)
    elif algorithm in DSTAR:
        graph.fig.canvas.mpl_connect('key_press_event', lambda event: on_key(event, graph))

    input("Hit return to end")     

if __name__ == "__main__":
    main()
