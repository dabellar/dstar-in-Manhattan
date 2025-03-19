'''dstar.py

    Appendix B: 
    D* search algorithm implementation.
    Follows the D* Lite pseudocode (Fig. 5) in
    Fast Replanning for Navigation in Unknown Terrain by Sven Koenig and Maxim Likhachev.
'''

import heapq
from manhattangrid import MGNode, ManhattanGrid

def h(node: MGNode, other: MGNode):
    '''Return the heuristic value (manhattan distance) for a node.'''
    return node.cost_to_go(other)
    
def c(graph: ManhattanGrid, node: MGNode, other: MGNode):
    '''Return the cost to connect two nodes.'''
    return graph.get_edge_cost(node, other)

def top_key(U: heapq):
        '''Return the top key in the priority queue.'''
        if len(U) > 0:
            return tuple(U[0][0])
        return (float('inf'), float('inf'))

def top(U: heapq):
        '''Return the top node in the priority queue.'''
        return heapq.heappop(U)[1]

def calculate_key(s: MGNode, curr: MGNode, km):
    '''Calculate the key for a node.'''
    k1 = min(s.g, s.rhs) + h(curr, s) + km
    k2 = min(s.g, s.rhs)
    return (k1, k2)

def init_dstar(graph: ManhattanGrid, U: heapq, km):
    '''Initialize the D* search algorithm.'''
    graph.goal.rhs = 0
    heapq.heappush(U, (calculate_key(graph.goal, graph.start, km), graph.goal))
    compute_shortest_path(graph, U, graph.start, km)
    return U, km


def update_vertex(graph: ManhattanGrid, U: heapq, u : MGNode, curr: MGNode, km):
        '''Update the vertex and U.'''
        if u != graph.goal:
            min_rhs = float('inf')
            for s in u.neighbors:
                min_rhs = min(min_rhs, c(graph, u, s) + s.g)
            u.rhs = min_rhs
        u_in_U = [node for node in U if u in node]
        if u_in_U:
            U.remove(u_in_U[0])
        if u.g != u.rhs:
            heapq.heappush(U, (calculate_key(u, curr, km), u))

def compute_shortest_path(graph: ManhattanGrid, U: heapq, s: MGNode, km):
        '''Compute the shortest path.'''
        while (s.rhs != s.g) or (top_key(U) < calculate_key(s, s, km)):
            k_old = top_key(U)
            u = top(U)
            if k_old < calculate_key(u, s, km):
                heapq.heappush(U, (calculate_key(u, s, km), u))
            elif u.g > u.rhs:
                u.g = u.rhs
                for par in u.parents:
                    update_vertex(graph, U, par, s, km)
            else:
                u.g = float('inf')
                update_vertex(graph, U, u, s, km)
                for s in u.parents:
                    update_vertex(graph, U, s, s, km)
        

def next_move(graph: ManhattanGrid, curr: MGNode):
    '''Get the next move in path.'''
    min_cost = float('inf')
    next_node = None
    if curr.rhs == float('inf'):
        print("No known path to goal")
        return None
    else:
        for s in curr.neighbors:
            cost = c(graph, curr, s) + s.g
            if cost < min_cost:
                min_cost = cost
                next_node = s
        if next_node:
            return next_node
        
def rescan(graph: ManhattanGrid, U: heapq, km):
    '''Rescan the path from the start to the goal.'''
    changed_edges = []
    for edge in graph.edges:
        if graph.blocked[edge]:
            changed_edges.append(edge)
    return changed_edges

def replan(graph: ManhattanGrid, U: heapq, curr: MGNode, km):
    '''Replan the path from the current node to the goal.'''
    if curr == graph.goal:
        return graph.goal, km
    else:
        last = curr
        next = next_move(graph, curr)
        if next is None:
            return None, km
        if graph.is_blocked(curr, next):
            next = curr
        changed = rescan(graph, U, km)
        km += h(last, next)
        for edge in changed:
            u, v = edge
            update_vertex(graph, U, u, curr, km)
        compute_shortest_path(graph, U, curr, km)
        return next, km
