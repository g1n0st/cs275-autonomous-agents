import numpy as np
from map import *
from utils import *
import heapq
import time

def _single_and_sorted_Q_base(map : EnvMap, start_pos, end_pos, H):
    start_id = map.get_quadtree_node(start_pos)
    end_id = map.get_quadtree_node(end_pos)

    close_set = set()

    came_from = {}
    came_from[start_id] = -1

    gscore = {start_id : 0}
    fscore = {start_id : distance(map.quadtree_node[start_id].actual_position(), end_pos)}
    open_set = []

    heapq.heappush(open_set, (fscore[start_id], start_id))
    
    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == end_id:
            break

        close_set.add(current)
        for neighbor in map.quadtree_node[current].conn:         
            tentative_g_score = gscore[current] + \
                                distance(map.quadtree_node[neighbor].actual_position(), 
                                     map.quadtree_node[current].actual_position())

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_set]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + H * distance(map.quadtree_node[neighbor].actual_position(), end_pos)
                heapq.heappush(open_set, (fscore[neighbor], neighbor))

    
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
            
    path = list(reversed(path))
    return path

def SingleQ(map : EnvMap, start_pos, end_pos):
    start_time = time.time()
    # "SingleQ" is the standard A* algorithm
    path = _single_and_sorted_Q_base(map, start_pos, end_pos, H = 1.0)
    end_time = time.time()
    print(f"SingleQ runtime={end_time-start_time}s")

    return path

def SortedQ(map : EnvMap, start_pos, end_pos):
    start_time = time.time()
    # "sortedQ" Keep heuristic part zero, thus it first visits the frontier node 
    # (frontier nodes are nodes in the boundary between visited and unvisited regions) 
    # which is closest to the start node
    path = _single_and_sorted_Q_base(map, start_pos, end_pos, H = 0.0)
    end_time = time.time()
    print(f"SingleQ runtime={end_time-start_time}s")

    return path

def _sorted_Q_expansion(map : EnvMap, end_pos):
    end_id = map.get_quadtree_node(end_pos)

    close_set = set()

    came_from = {}
    came_from[end_id] = -1

    fscore = {end_id : distance(map.quadtree_node[end_id].actual_position(), end_pos)}
    open_set = []

    heapq.heappush(open_set, (fscore[end_id], end_id))
    
    while open_set:
        current = heapq.heappop(open_set)[1]

        if map.quadtree_node[current].level >= Lt:
            midpoint = current
            break

        close_set.add(current)
        for neighbor in map.quadtree_node[current].conn:         
            tentative_score = distance(map.quadtree_node[neighbor].actual_position(), 
                                     end_pos)

            if neighbor in close_set and tentative_score >= fscore.get(neighbor, 0):
                continue
                
            if  tentative_score < fscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_set]:
                came_from[neighbor] = current
                fscore[neighbor] = tentative_score
                heapq.heappush(open_set, (fscore[neighbor], neighbor))

    
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]

    return map.quadtree_node[midpoint].actual_position(), path

def _PmultiQ_base(map : EnvMap, start_pos, end_pos):
    start_id = map.get_quadtree_node(start_pos)
    end_id = map.get_quadtree_node(end_pos)

    close_set = set()

    came_from = {}
    came_from[start_id] = -1

    gscore = {start_id : 0}
    fscore = {start_id : distance(map.quadtree_node[start_id].actual_position(), end_pos)}
    open_set = [[] for l in range(map.quadtree_level)]

    heapq.heappush(open_set[map.quadtree_node[start_id].level], (fscore[start_id], start_id))
    
    while True:
        for l in reversed(range(map.quadtree_level)):
            if len(open_set[l]) > 0:
                current = heapq.heappop(open_set[l])[1]
                break

        if current == end_id:
            break

        close_set.add(current)
        for neighbor in map.quadtree_node[current].conn:
            neighbor_level = map.quadtree_node[neighbor].level
            tentative_g_score = gscore[current] + \
                                distance(map.quadtree_node[neighbor].actual_position(), 
                                     map.quadtree_node[current].actual_position())

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_set[neighbor_level]]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + 1.0 * distance(map.quadtree_node[neighbor].actual_position(), end_pos)
                heapq.heappush(open_set[neighbor_level], (fscore[neighbor], neighbor))

    
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
            
    path = list(reversed(path))
    
    return path

def PmultiQ(map : EnvMap, start_pos, end_pos, enable_expansion = True):
    # "PmultiQ", is a prioritized MultiQ scheme, 
    # which tries to visit the largest frontier node first 
    # and therefore exhibits the fastest growth speed among the four methods.
    start_time = time.time()
    if enable_expansion:
        mid_pos, end_path = _sorted_Q_expansion(map, end_pos)
        path = _PmultiQ_base(map, start_pos, mid_pos)
        # By linking these two paths together, we obtain a complete path to the target.
        path.pop()
        path += end_path
    else:
        path = _PmultiQ_base(map, start_pos, end_pos)
    end_time = time.time()
    print(f"PmultiQ(enable_expansion={enable_expansion}) runtime={end_time-start_time}s")

    return path