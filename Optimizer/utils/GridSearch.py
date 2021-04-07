from heapq import *
import matplotlib.pyplot as plt
import numpy as np

from .LineSegment2d import LineSegment2d
from .Node2d import Node2d
from .Vec2d import Vec2d

class GridSearch:
    def __init__(self, node_radius, xy_res):
        self.node_radius = node_radius
        self.xy_res = xy_res
        self.XYBounds = []
        self.obstacles_segments = []
        self.max_grid_x = 0
        self.max_grid_y = 0
    
    def euclidDistance(self, x1, y1, x2, y2):
        return ((x1-x2)**2 + (y1-y2)**2)**(1/2)
    
    def checkConstraints(self, next_node, curr_node):
        if next_node.grid_x < 0 or next_node.grid_x > self.max_grid_x or \
        next_node.grid_y < 0 or next_node.grid_y > self.max_grid_y:
            return False
        
        if len(self.obstacles_segments) == 0:
            return True
        
        for obstacle in self.obstacles_segments:
            for obstacle_seg in obstacle:
                if obstacle_seg.distanceToPoint(Vec2d(next_node.x, next_node.y)) < self.node_radius:
                    return False
                if LineSegment2d(Vec2d(curr_node.x, curr_node.y), Vec2d(next_node.x, next_node.y)).isIntersect(obstacle_seg):
                    return False
        return True
    
    def generateNextNodes(self, curr_node):
        path_cost = curr_node.path_cost
        x = curr_node.x
        y = curr_node.y
        diag_dist = 2**(1/2)
        
        next_nodes = []
        up = Node2d(x, y + self.xy_res, self.xy_res, self.XYBounds)
        up.setPathCost(path_cost + 1)
        next_nodes.append(up)
        
        up_right = Node2d(x + self.xy_res, y + self.xy_res, self.xy_res, self.XYBounds)
        up_right.setPathCost(path_cost + diag_dist)
        next_nodes.append(up_right)
        
        right = Node2d(x + self.xy_res, y, self.xy_res, self.XYBounds)
        right.setPathCost(path_cost + 1)
        next_nodes.append(right)
        
        down_right = Node2d(x + self.xy_res, y - self.xy_res, self.xy_res, self.XYBounds)
        down_right.setPathCost(path_cost + diag_dist)
        next_nodes.append(down_right)
        
        down = Node2d(x, y - self.xy_res, self.xy_res, self.XYBounds)
        down.setPathCost(path_cost + 1)
        next_nodes.append(down)
        
        down_left = Node2d(x - self.xy_res, y - self.xy_res, self.xy_res, self.XYBounds)
        down_left.setPathCost(path_cost + diag_dist)
        next_nodes.append(down_left)
        
        left = Node2d(x - self.xy_res, y, self.xy_res, self.XYBounds)
        left.setPathCost(path_cost + 1)
        next_nodes.append(left)
        
        up_left = Node2d(x - self.xy_res, y + self.xy_res, self.xy_res, self.XYBounds)
        up_left.setPathCost(path_cost + diag_dist)
        next_nodes.append(up_left)
        
        return next_nodes
    
    def generateAStartPath(self, s_x, s_y, e_x, e_y, XYBounds, obstacles_vertices):
        self.obstacles_segments = self.getObstaclesSegments(obstacles_vertices)
        self.XYBounds = XYBounds
        self.max_grid_x = (XYBounds[1] - XYBounds[0]) // self.xy_res
        self.max_grid_y = (XYBounds[3] - XYBounds[2]) // self.xy_res
        
        start_node = Node2d(s_x, s_y, self.xy_res, XYBounds)
        end_node = Node2d(e_x, e_y, self.xy_res, XYBounds)
        final_node = None
        
        open_set = {}
        close_set = {}
        open_pq = []
        
        open_set[start_node.index] = start_node
        heappush(open_pq, (start_node.getCost(), start_node.index))
        
        explored_num = 0
        while len(open_pq) != 0:
            curr_node = open_set[heappop(open_pq)[1]]
            if curr_node == end_node:
                final_node = curr_node
                break
            
            close_set[curr_node.index] = curr_node
            next_nodes = self.generateNextNodes(curr_node)
            for next_node in next_nodes:
                if (not self.checkConstraints(next_node, curr_node)):
                    continue
                if close_set.get(next_node.index) is not None:
                    continue
                if open_set.get(next_node.index) is None:
                    explored_num += 1
                    next_node.setHeuCost(self.euclidDistance(next_node.grid_x, next_node.grid_y,
                                                            end_node.grid_x, end_node.grid_y))
                    next_node.pre_node = curr_node
                    open_set[next_node.index] = next_node
                    heappush(open_pq, (next_node.getCost(), next_node.index))
        
        if final_node is None:
            return None
        else:
            curr_node = final_node
            path_points = []
            while curr_node is not None: 
                path_points.append(Vec2d(curr_node.x, curr_node.y))
                curr_node = curr_node.pre_node  
            
            return path_points[::-1]
        
    def plot_path(self, figsize, path_points, optimized_trajectory=None):
        plt.figure(figsize=figsize)
        plt.xlim(self.XYBounds[0], self.XYBounds[1])
        plt.ylim(self.XYBounds[2], self.XYBounds[3])
        plt.xticks(np.arange(self.XYBounds[0], self.XYBounds[1], self.xy_res))
        plt.yticks(np.arange(self.XYBounds[2], self.XYBounds[3], self.xy_res))
        x_vec = list(map(lambda vec: vec.x, path_points))
        y_vec = list(map(lambda vec: vec.y, path_points))
        plt.plot(x_vec, y_vec)
        if optimized_trajectory:
            x_vec = list(map(lambda vec: vec.x, optimized_trajectory))
            y_vec = list(map(lambda vec: vec.y, optimized_trajectory))
            plt.scatter(x_vec, y_vec, alpha=0.7, c='black')
        for obstacle in self.obstacles_segments:
            for obstacle_seg in obstacle:
                plt.plot([obstacle_seg.start.x, obstacle_seg.end.x],
                        [obstacle_seg.start.y, obstacle_seg.end.y], c='r')
        plt.grid()
        plt.show()
    
    @staticmethod
    def getObstaclesSegments(obstaclesVerticesVectors):
        obstacles = []
        for verticesVector in obstaclesVerticesVectors:
            obstacle_segments = []
            n = len(verticesVector)
            for i in range(n):
                if i < n-1:
                    obstacle_segments.append(LineSegment2d(verticesVector[i], verticesVector[i+1]))
                elif n > 2:
                    obstacle_segments.append(LineSegment2d(verticesVector[i], verticesVector[0]))
            obstacles.append(obstacle_segments)
        return obstacles