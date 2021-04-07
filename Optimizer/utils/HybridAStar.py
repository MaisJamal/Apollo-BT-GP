import copy
import heapq as hq
import math
import matplotlib.pyplot as plt
import numpy as np


class HybridAStar:
    def __init__(self, min_x, max_x, min_y, max_y, obstacle=(), resolution=1, vehicle_length=2):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.resolution = resolution
        self.vehicle_length = vehicle_length

        self.obstacles = set(obstacle)

    def euc_dist(self, position, target):
        output = np.sqrt((position[0] - target[0]) ** 2
                         + (position[1] - target[1]) ** 2
                         + (math.radians(position[2]) - math.radians(target[2])) ** 2)
        return float(output)

    """
    For each node n, we need to store:
    (discrete_x, discrete_y, heading angle theta),
    (continuous x, continuous y, heading angle theta)
    cost g, f,
    path [(continuous x, continuous y, continuous theta),...]
    start: discrete (x, y, theta)
    end: discrete (x, y, theta)
    sol_path = [(x1,y1,theta1),(x2,y2,theta2), ...]
    """

    def find_path(self, start, end):
        steering_inputs = [-40, 0, 40]
        cost_steering_inputs = [0.1, 0, 0.1]

        speed_inputs = [-1, 1]
        cost_speed_inputs = [1, 0]

        start = (float(start[0]), float(start[1]), float(start[2]))
        end = (float(end[0]), float(end[1]), float(end[2]))
        # The above 2 are in discrete coordinates

        open_heap = []  # element of this list is like (cost,node_d)
        open_diction = {}  # element of this is like node_d:(cost,node_c,(parent_d,parent_c))

        visited_diction = {}  # element of this is like node_d:(cost,node_c,(parent_d,parent_c))

        cost_to_neighbour_from_start = 0

        # Here a heap is chosen. (cost, path) is a tuple which is pushed in 
        # open_set_sorted. The main advantage is that as more (cost, path) are 
        # added to this open_set, heap automatically sorts it and this first 
        # element is automatically the lowest cost one
        # Here path is [(),()....] where each () has (discrete,continuous) for a node
        # for path normal appending is done. If you use heap there, the elements
        # get sorted and we don't want that. We want to preserve the order in 
        # which we move for start to destination node

        hq.heappush(open_heap, (cost_to_neighbour_from_start + self.euc_dist(start, end), start))

        open_diction[start] = (cost_to_neighbour_from_start + self.euc_dist(start, end), start, (start, start))

        while len(open_heap) > 0:
            # choose the node that has minimum total cost for exploration
            # print(open_set_sorted)
            chosen_node_total_cost, chosen_d_node = open_heap[0]
            chosen_c_node = open_diction[chosen_d_node][1]

            visited_diction[chosen_d_node] = open_diction[chosen_d_node]

            # print(self.euc_dist(chosen_path_last_element[0],end))

            if self.euc_dist(chosen_d_node, end) < 1:
                rev_final_path = [end]  # reverse of final path
                node = chosen_d_node
                while True:
                    open_node_contents = visited_diction[node]  # (cost,node_c,(parent_d,parent_c))                   
                    parent_of_node = open_node_contents[2][1]

                    rev_final_path.append(parent_of_node)
                    node = open_node_contents[2][0]
                    if node == start:
#                         rev_final_path.append(start)
                        break
                return rev_final_path[::-1]
            
            hq.heappop(open_heap)
            for i in range(0, 3):
                for j in range(0, 2):
                    delta = steering_inputs[i]
                    velocity = speed_inputs[j]

                    cost_to_neighbour_from_start = chosen_node_total_cost - self.euc_dist(chosen_d_node, end)

                    neighbour_x_cts = chosen_c_node[0] + (velocity * math.cos(math.radians(chosen_c_node[2])))
                    neighbour_y_cts = chosen_c_node[1] + (velocity * math.sin(math.radians(chosen_c_node[2])))
                    neighbour_theta_cts = math.radians(chosen_c_node[2]) + (
                                velocity * math.tan(math.radians(delta)) / (float(self.vehicle_length)))

                    neighbour_theta_cts = math.degrees(neighbour_theta_cts)

                    neighbour_x_d = round(neighbour_x_cts)
                    neighbour_y_d = round(neighbour_y_cts)
                    neighbour_theta_d = round(neighbour_theta_cts)

                    neighbour = ((neighbour_x_d, neighbour_y_d, neighbour_theta_d),
                                 (neighbour_x_cts, neighbour_y_cts, neighbour_theta_cts))

                    if (((neighbour_x_d, neighbour_y_d) not in self.obstacles) and 
                            (neighbour_x_d >= self.min_x) and (neighbour_x_d <= self.max_x) and 
                            (neighbour_y_d >= self.min_y) and (neighbour_y_d <= self.max_y)):

                        heurestic = self.euc_dist((neighbour_x_d, neighbour_y_d, neighbour_theta_d), end)
                        cost_to_neighbour_from_start = abs(velocity) + cost_to_neighbour_from_start + \
                                                       cost_steering_inputs[i] + cost_speed_inputs[j]

                        total_cost = heurestic + cost_to_neighbour_from_start

                        # If the cost of going to this successor happens to be more
                        # than an already existing path in the open list to this successor,
                        # skip this successor                   
                        skip = False
                        # print(open_set_sorted)
                        # If the cost of going to this successor happens to be more
                        # than an already existing path in the open list to this successor,
                        # skip this successor
                        found_lower_cost_path_in_open = 0

                        if neighbour[0] in open_diction:
                            if total_cost > open_diction[neighbour[0]][0]:
                                skip = True
                            elif neighbour[0] in visited_diction:
                                if total_cost > visited_diction[neighbour[0]][0]:
                                    found_lower_cost_path_in_open = 1
                        if not skip and found_lower_cost_path_in_open == 0:
                            hq.heappush(open_heap, (total_cost, neighbour[0]))
                            open_diction[neighbour[0]] = (total_cost, neighbour[1], (chosen_d_node, chosen_c_node))
                            
        print("Did not find the goal - it's unattainable.")
        return []
    
    
sx, sy, stheta= -2, -0, 0
gx, gy, gtheta = 2, 0, 0 #2,4,0 almost exact

#create obstacles
obstacle = []

for i in range(3):
    obstacle.append((0,i))
    obstacle.append((0,-i))

ox, oy = [], []
for (x,y) in obstacle:
    ox.append(x)
    oy.append(y)

plt.figure(figsize=(14,14))
plt.plot(ox, oy, ".k")
plt.plot(sx, sy, "xr")
plt.plot(gx, gy, "xb")
plt.grid(True)
plt.axis("equal")

path = HybridAStar(-6, 6, -6, 6, obstacle=obstacle, \
    resolution=1, vehicle_length=2).find_path((sx,sy,stheta), (gx,gy,gtheta))

rx, ry = [], []
for node in path:
    rx.append(node[0])
    ry.append(node[1])

plt.plot(rx, ry, "-r")
plt.show()