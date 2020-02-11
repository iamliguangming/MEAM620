import heapq
from heapq import heappush, heappop  # Recommended.
import numpy as np

from flightsim.world import World
from proj1_2.code.occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    occ_map.create_map_from_world
    Q = []
    cost_to_come = np.full((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2]),np.inf)
    heuristic = np.zeros((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2]))
    
    parent = np.zeros((occ_map.map.shape[0],occ_map.map.shape[1],occ_map.map.shape[2],3))
    
    
    if astar == False:
        for i in range(occ_map.map.shape[0]):
            for j in range(occ_map.map.shape[1]):
                for k in range(occ_map.map.shape[2]):
                    if  occ_map.map[i,j,k] == False:
                        heapq.heappush(Q,(cost_to_come[i,j,k],[i,j,k]))
                        # heuristic[i,j,k] = np.sqrt((goal_index[0]-i)**2+(goal_index[1]-j)**2 + (goal_index[2]-k)**2)
        Q.remove((cost_to_come[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
        cost_to_come[start_index[0],start_index[1],start_index[2]] = 0
        Q.append((cost_to_come[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
    
                                                               
        # heapq.heapreplace(Q, (cost_to_come[start_index[0],start_index[1]],[start_index[0],start_index[1]]))
        #Done with initialization
        #Following is the searching process
        while (cost_to_come[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]) in Q and cost_to_come.min() < np.inf:
            u_indicies = min(Q)[1]
            u_x = u_indicies[0]
            u_y = u_indicies[1]
            u_z = u_indicies[2]
            heapq.heapify(Q)
            heapq.heappop(Q)
            for i in range(-1,2):
                for j in range(-1,2):
                    for k in range(-1,2):
                        if i==0 and j==0 and k==0:
                            pass
                        elif not occ_map.is_valid_index([u_x+i,u_y+j,u_z+k]) or occ_map.is_occupied_index([u_x+i,u_y+j,u_z+k]):
                            pass
                        else:
                            d = cost_to_come[u_x,u_y,u_z]+1
                            if d < cost_to_come[u_x+i,u_y+j,u_z+k]:
                                Q.remove((cost_to_come[u_x+i,u_y+j,u_z+k],[u_x+i,u_y+j,u_z+k]))
                                cost_to_come[u_x+i,u_y+j,u_z+k] = d
                                Q.append((d,[u_x+i,u_y+j,u_z+k]))
                                heapq.heapify(Q)
                                parent[u_x+i,u_y+j,u_z+k,:] = [u_x,u_y,u_z]
                # print([u_x,u_y,u_z])
        current_X = goal_index[0]
        current_Y = goal_index[1]
        current_Z = goal_index[2]
        
    elif astar == True:
        for i in range(occ_map.map.shape[0]):
            for j in range(occ_map.map.shape[1]):
                for k in range(occ_map.map.shape[2]):
                    if  occ_map.map[i,j,k] == False:
                        heapq.heappush(Q,(cost_to_come[i,j,k],[i,j,k]))
                        heuristic[i,j,k] = np.sqrt((goal_index[0]-i)**2+(goal_index[1]-j)**2 + (goal_index[2]-k)**2)
                        
                        
        f = cost_to_come + heuristic
        Q.remove((f[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
        cost_to_come[start_index[0],start_index[1],start_index[2]] = 0
        f = cost_to_come + heuristic
        Q.append((f[start_index[0],start_index[1],start_index[2]],[start_index[0],start_index[1],start_index[2]]))
    
                                                               
        # heapq.heapreplace(Q, (cost_to_come[start_index[0],start_index[1]],[start_index[0],start_index[1]]))
        #Done with initialization
        #Following is the searching process
        while (f[goal_index[0],goal_index[1],goal_index[2]],[goal_index[0],goal_index[1],goal_index[2]]) in Q and f.min() < np.inf:
            u_indicies = min(Q)[1]
            u_x = u_indicies[0]
            u_y = u_indicies[1]
            u_z = u_indicies[2]
            heapq.heapify(Q)
            heapq.heappop(Q)
            for i in range(-1,2):
                for j in range(-1,2):
                    for k in range(-1,2):
                        if i==0 and j==0 and k==0:
                            pass
                        elif not occ_map.is_valid_index([u_x+i,u_y+j,u_z+k]) or occ_map.is_occupied_index([u_x+i,u_y+j,u_z+k]):
                            pass
                        else:
                            d = cost_to_come[u_x,u_y,u_z]+1
                            if d < cost_to_come[u_x+i,u_y+j,u_z+k]:
                                Q.remove((f[u_x+i,u_y+j,u_z+k],[u_x+i,u_y+j,u_z+k]))
                                cost_to_come[u_x+i,u_y+j,u_z+k] = d
                                f = cost_to_come + heuristic
                                Q.append((f[u_x+i,u_y+j,u_z+k],[u_x+i,u_y+j,u_z+k]))
                                heapq.heapify(Q)
                                parent[u_x+i,u_y+j,u_z+k,:] = [u_x,u_y,u_z]
                # print([u_x,u_y,u_z])
        current_X = goal_index[0]
        current_Y = goal_index[1]
        current_Z = goal_index[2]
        
        
        
    path = []
    
    
    while [current_X,current_Y,current_Z] != [start_index[0],start_index[1],start_index[2]]:
        path.insert(0,occ_map.index_to_metric_center([current_X,current_Y,current_Z]))
        [current_X,current_Y,current_Z] = parent[int(current_X),int(current_Y),int(current_Z)]
        
    return np.asarray(path)
        
        
        
                
        
        
        
                
            
    

    # if astar == False:
    #     pass
    
    
    

    return None

