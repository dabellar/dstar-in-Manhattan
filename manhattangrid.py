'''manhattangrid.py
    
    Appendix A: 
    Leverage OSMnx and user-created MGNode to create a Manhattan grid graph
    representation.
'''

import osmnx as ox
import matplotlib.pyplot as plt
import random
from utils import Color

# Define colors
RED = Color.RED
DARK_RED = Color.DARK_RED
WHITE = Color.WHITE
PURPLE = Color.PURPLE
BLACK = Color.BLACK
GREEN = Color.GREEN
BLUE = Color.BLUE
SKY = Color.SKY

class MGNode:
    def __init__(self, x, y):
        '''Initialize the MGNode object.'''
        self.x = x
        self.y = y
        self.neighbors = set()
        self.parents = set()
        # astar variables
        self.done = False
        self.seen = False
        self.parent = None
        self.reach = float('inf')
        self.cost = float('inf')
        # dstar variables
        self.g = float('inf')
        self.rhs = float('inf')
        self.curr = False

    def reset(self):
        '''Reset the node for a new search.'''
        self.done = False
        self.seen = False
        self.parent = None
        self.reach = 0
        self.cost = float('inf')

    def cost_to_go(self, other: 'MGNode'):
        '''Return the cost to go (manhattan distance) from this node to another node.'''
        manhattan_distance = abs(self.x - other.x) + abs(self.y - other.y)
        return manhattan_distance
    
    def __lt__(self, other: 'MGNode'):
        '''Define the less than operator for the node.'''
        return (self.cost) < (other.cost)
    
    def __iter__(self):
        '''Return an iterator over the node's attributes.'''
        return iter((self.x, self.y, self.neighbors))
    
    def __str__(self):
        '''Return a string representation of the node.'''
        return f"MGNode:\nx={self.x}, y={self.y}\nnumber of neighbors={len(self.neighbors)}\nnumber of parents={len(self.parents)}\n"

class ManhattanGrid:
    def __init__(self, *args):
        '''Initialize the ManhattanGrid object.'''
        if len(args) == 1:
            self.place_name = args[0]
            self.oxGraph = ox.graph_from_place(self.place_name, network_type='drive', simplify=True)
        else:
            # bounding box:  min Longitude , min Latitude , max Longitude , max Latitude 
            self.bbox = (args[0], args[1], args[2], args[3])
            self.oxGraph = ox.graph_from_bbox(self.bbox, network_type='drive', simplify=True)
        self.mgnodes = {} # nodeId: MGNode
        self.edges = {} # (mgnode1, mgnode2): current edge cost
        self.blocked = {} # (mgnode1, mgnode2): blocked or not
        self.base_costs = {}  # (mgnode1, mgnode2): Base costs (edge lengths)
        self.start = None
        self.goal = None
        self.colors_n = {} # (x, y): color
        self.colors_e = {} # (node1, node2): color
        self.fig = None
        self.ax = None

        # format nodes and extract edges/edge lengths
        self._format_mgnodes()
        self._extract_edges()

        # Create/activate a figure, clear, color
        self._initialize_plt_graph()
            
    def _format_mgnodes(self):
        '''Format MGNodes from ox coordinates and neighbors.'''
        for node in self.oxGraph.nodes:
            x = self.oxGraph.nodes[node]['x']
            y = self.oxGraph.nodes[node]['y']
            self.mgnodes[node] = MGNode(x, y)
            self.color(x, y, BLACK)
        for mg in self.mgnodes:
            neighbors = list(self.oxGraph.neighbors(mg))
            for nbr in neighbors:
                if nbr in self.mgnodes:
                    self.mgnodes[mg].neighbors.add(self.mgnodes[nbr])
                    self.mgnodes[nbr].parents.add(self.mgnodes[mg])
    
    def _extract_edges(self):
        '''Extract edges from the graph.'''
        for u, v, data in self.oxGraph.edges(data=True):
            if u in self.mgnodes and v in self.mgnodes:
                base_cost = data['length']  # Use length as base cost
                u_node = self.mgnodes[u]
                v_node = self.mgnodes[v]
                self.edges[(u_node, v_node)] = base_cost
                self.blocked[(u_node, v_node)] = False
                self.base_costs[(u_node, v_node)] = base_cost
                self._color_edge(u_node, v_node, BLUE)

    def block_edge(self):
        '''Randomly block edges.'''
        for edge in self.edges:
            u, v = edge
            edge2 = (v, u) if (v, u) in self.edges else None
            rand = random.uniform(0, 1)
            if rand < 0.03:
                old_blocked = self.blocked[edge]
                self.blocked[edge] = not old_blocked
                self.edges[edge] = float('inf') if self.blocked[edge] else self.base_costs[edge]
                if edge2 is not None:
                    old_blocked2 = self.blocked[edge2]
                    self.blocked[edge2] = not old_blocked2
                    self.edges[edge2] = float('inf') if self.blocked[edge2] else self.base_costs[edge2]


    def is_blocked(self, node1: MGNode, node2: MGNode):
        '''Check if an edge is blocked.'''
        return self.blocked.get((node1, node2), False) # Return False if edge not found
    
    def get_edge_cost(self, node1: MGNode, node2: MGNode):
        '''Get the current cost of an edge.'''
        return self.edges.get((node1, node2), float('inf')) # Return inf if edge not found
    
    def get_node(self, mgnode: MGNode):
        '''Get the MGNode object from the graph.'''
        for node in self.mgnodes.values():
            if node == mgnode:
                return node
        return None
    
    def set_start(self, node: MGNode):
        '''Set the start node.'''
        self.start = node
        self.color(node.x, node.y, DARK_RED)

    def set_goal(self, node: MGNode):
        '''Set the goal node.'''
        self.goal = node
        self.color(node.x, node.y, DARK_RED)

    def _initialize_plt_graph(self):
        '''Initialize and show the plt graph.'''
        self.fig, self.ax = plt.subplots()
        plt.clf()
        self.ax = plt.axes()
        self.ax.axis('off')

        # Plot edges
        for line in self.edges:
            x, y = (line[0].x, line[1].x), (line[0].y, line[1].y)
            self.ax.plot(x, y, color=BLUE, linewidth=0.75)

        manager = plt.get_current_fig_manager()
        manager.full_screen_toggle()

        # Show the grid.
        self.show()

    def color(self, x, y, color):
        '''Color the grid node at (x, y) with given color.'''
        self.colors_n[(x, y)] = color

    def _color_edge(self, node1, node2, color):
            '''Color the edge at (node1, node2) with the given color.'''
            self.colors_e[node1, node2] = color

    def show(self, wait=False):
        '''Show the grid with colors (taken from 133B HW1).'''
        # Update the edge colors
        for (node1, node2), color in self.colors_e.items():
            x, y = (node1.x, node2.x), (node1.y, node2.y)
            plt.plot(x, y, color=color, linewidth=0.75)
        # Update the node colors
        for (x, y), color in self.colors_n.items():
            plt.plot(x, y, 'o', markersize=6, color=color)
        plt.show(block=False)
        if isinstance(wait, float):
            plt.pause(max(0.000025, wait))
        elif wait:
            plt.pause(0.000025)
            input('Hit return to continue' if wait is True else wait)
        else:
            plt.pause(0.000025)

    def plot_path(self, path):
        '''Plot the path on the grid.'''
        if path:
            print("Path found")
            for mg in path:
                path_x = [mg.x for mg in path]
                path_y = [mg.y for mg in path]
                self.ax.plot(path_x, path_y, color=RED, linewidth=6, marker='o', markersize=6)
                self.color(self.start.x, self.start.y, DARK_RED)
                self.color(self.goal.x, self.goal.y, DARK_RED)
            self.show()
        else:
            print("No path found.")

    def update_visualization(self, path):
        '''Update the visualization of the graph.'''
        # Plot edges
        for edge, cost in self.edges.items():
            node1, node2 = edge
            color = RED if self.blocked[edge] else BLUE
            self._color_edge(node1, node2, color)

        # Plot nodes
        for node in self.mgnodes.values():
            if node.curr:
                self.color(node.x, node.y, PURPLE)
            else:
                self.color(node.x, node.y, SKY)
        self.color(self.start.x, self.start.y, DARK_RED)
        self.color(self.goal.x, self.goal.y, GREEN)

        self.show(wait=0.0000025)

    def save(self, filename):
        '''Save the grid to a file.'''
        plt.savefig(filename)
        print(f"Grid saved to {filename}")

    