'''astar.py
    
    A* search algorithm implementation.
'''

import heapq
from manhattangrid import MGNode, ManhattanGrid

class AStar:
    def __init__(self, graph: ManhattanGrid):
        '''Initialize the AStar object.'''
        self.graph = graph
        self.start = graph.start
        self.goal = graph.goal
        self.path = []
        self.reset()
    
    def reset(self):
        '''Reset the A* search tree for a new search.'''
        for node, mg in self.graph.mgnodes.items():
            mg.reset()
        
    def cost_to_connect(self, mgnode1: MGNode, mgnode2: MGNode):
        '''Return the cost to connect two nodes.'''
        return self.graph.get_edge_cost(mgnode1, mgnode2)
    
    
    def find_path(self, show=None):
        '''Find the path from start to goal using A* (From 133B HW3 astar.py).'''
        # Clear the A* search tree information.
        self.reset()

        # Prepare the still empty *sorted* on-deck queue.
        onDeck = []

        # Begin with the start node on-deck.
        self.start.done = False
        self.start.seen = True
        self.start.parent = None
        self.start.reach = 0
        self.start.cost = self.start.cost_to_go(self.goal)
        heapq.heappush(onDeck, self.start)

        print("Finding path using A* algorithm...")
        # Continually expand/build the search tree.
        while True:
            # Show the grid
            if show:
                show()
            # Make sure we have something pending in the on-deck queue.
            if not (len(onDeck) > 0):
                return None

            # Grab the next node (first on deck).
            mgnode = heapq.heappop(onDeck)

            # Mark this node as done and check if the goal is thereby done.
            mgnode.done = True
            if self.goal.done:
                break

            # Add the neighbors to the on-deck queue (or update).
            for neighbor in mgnode.neighbors:
                # Skip if already done.
                if neighbor.done:
                    continue

                # Compute the cost to reach the neighbor via this new path.
                creach = mgnode.reach + self.cost_to_connect(mgnode, neighbor)

                # Just add to on-deck if not yet seen (in correct order).
                if not neighbor.seen:
                    neighbor.seen = True
                    neighbor.parent = mgnode
                    neighbor.reach = creach
                    neighbor.cost = neighbor.cost_to_go(self.goal)
                    heapq.heappush(onDeck, neighbor)
                    continue

                # Skip if the previous path to reach (cost) was same or better.
                if creach < neighbor.reach:
                    continue

                # Update the neighbor's connection and resort the on-deck queue.
                neighbor.parent = mgnode
                neighbor.reach = creach
                onDeck.remove(neighbor)
                heapq.heappush(onDeck, neighbor)

        # Build the path.
        path = [self.goal]
        while path[0].parent is not None:
            path.insert(0, path[0].parent)
        # add path
        self.path = path
        # Return the path.
        return path