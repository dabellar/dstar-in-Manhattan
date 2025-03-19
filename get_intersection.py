'''get_intersection.py
    
    This is left over from the first iteration of the project. It is not used in the final
    implementation, but it is used to get the intersection node id from the street names.
'''
import osmnx as ox

class MGNode:
    def __init__(self, node):
        '''Initialize the MGNode object.'''
        self.node = node # type: int
        self.x = None
        self.y = None
        self.neighbors = set()

class ManhattanGrid:
    def __init__(self, place_name):
        '''Initialize the ManhattanGrid object.'''
        self.place_name = place_name
        self.network_type = 'drive'
        self.mgnodes = []
        self.street_to_nodes = {}
        self.graph = ox.graph_from_place(place_name, network_type='drive', simplify=True)
        self.edges = ox.graph_to_gdfs(self.graph, nodes=False, edges=True)
        self._formatNodes()

    def get_nodes(self):
        '''Get the nodes in the graph.'''
        return list(self.graph.nodes)

    def get_neighbors(self, node):
        '''Get the neighbors of a node'''
        return list(self.graph.neighbors(node))

    def get_node_coordinates(self, node):
        '''Get the x and y coordinates of a node.'''
        return self.graph.nodes[node]['x'], self.graph.nodes[node]['y']

    def _formatNodes(self):
        '''Convert the nodes in the graph to MGNodes for easier algorithm use.'''
        node_dict = {}
        for node in self.get_nodes():
            new_node = MGNode(node)
            new_node.x, new_node.y = self.get_node_coordinates(node)
            self.mgnodes.append(new_node)
            node_dict[node] = new_node
        for node in self.mgnodes:
            neighbors = self.get_neighbors(node.node)
            for neighbor in neighbors:
                if neighbor in node_dict: 
                    node.neighbors.add(node_dict[neighbor])

    def get_intersection_node(self, ave, street):
        '''Get the node corresponding to an intersection.'''
        name_series = self.edges['name'].sort_index()
        if not self.street_to_nodes:
            for index, val in name_series.items():
                self.street_to_nodes.setdefault(str(val), set()).add(index[0])
                self.street_to_nodes.setdefault(str(val), set()).add(index[1])
        ave_nodes = set(self.street_to_nodes.get(ave, []))
        street_nodes = set(self.street_to_nodes.get(street, []))
        intersection_nodes = ave_nodes.intersection(street_nodes)
        if intersection_nodes:
            found_inter = [mgnode for mgnode in self.mgnodes if mgnode.node in intersection_nodes]
            return found_inter.pop()  # Return the first intersecting node
        return None
    
def main():
    # Initialize the graph
    graph = ManhattanGrid(place_name="Manhattan, New York, USA")
    print("Manhattan, NY Graph initialized.")

    # Hardcoded start and goal intersections 
    start_streets = ["Columbus Circle", "Central Park West"]
    goal_streets = ["Tunnel Approach Street", "East 37th Street"]
    start_inter = graph.get_intersection_node(start_streets[0], start_streets[1])
    goal_inter = graph.get_intersection_node(goal_streets[0], goal_streets[1])
    print(f"Start intersection: {start_streets[0]} and {start_streets[1]}: {start_inter.node}\nGoal intersection: {goal_streets[0]} and {goal_streets[1]}: {goal_inter.node}")

    # graphs
    # graph = ManhattanGrid("Manhattan, New York, USA") # full Manhattan map
    # graph = ManhattanGrid(-73.9709,40.7782,-73.9475,40.801)  # bounding box for 97th transverse
    # graph = ManhattanGrid(-73.9654,40.7887,-73.9214,40.8656) # bounding box for above 96th
    # graph = ManhattanGrid(-74.0083,40.7438,-73.9588,40.7721) # bounding box for below 59th
    # graph = ManhattanGrid(-73.9838,40.762,-73.9477,40.8038) # bounding box for around central park

    # starts and goals
    # start = graph.mgnodes[4347550065] # Columbus Circle and Central Park West
    # goal = graph.mgnodes[42449689] # Tunnel Approach Street and East 37th Street
    # start = graph.mgnodes[1762221470] # West 191st Street and Saint Nicholas Avenue

if __name__ == "__main__":
    main()