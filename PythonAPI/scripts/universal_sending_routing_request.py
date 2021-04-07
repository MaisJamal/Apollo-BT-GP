#!/usr/bin/env python3

from cyber_py3 import cyber, cyber_time

import math
import time
import os
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles


class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
        self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
        
      #  self.reader_node = cyber.Node("reader")
      #  self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)
        
        

if __name__ == '__main__':
    apollo_test = ApolloFeatures()

    time.sleep(2.0)
    # file should contain 2 colomns, one for X and the other for Y coordinates of the requested route.
    with open('PythonAPI/scripts/visiting_points.txt') as f:
        visiting_points = [[float(x) for x in line.split()] for line in f]
        
   
    with_priority = True
    """    
    msg = RoutingRequest()
    msg.header.module_name = 'dreamview'
    msg.header.sequence_num = 0
    
    
    if with_priority :
        priority = []
        with open('PythonAPI/scripts/routing_with_priority.txt') as h:
            line = h.readline()
            for i in line.split(): 
                if i.isdigit() == True:
                        priority.append(int(i)) 
        print(priority)
        for i in range(len(priority)):
            x = visiting_points[priority[i]][0]
            y = visiting_points[priority[i]][1]
        
            waypoint = msg.waypoint.add()
            waypoint.pose.x = float(x)
            waypoint.pose.y = float(y)
    else:
        for i in range(len(visiting_points)):
            x = visiting_points[i][0]
            y = visiting_points[i][1]
        
            waypoint = msg.waypoint.add()
            waypoint.pose.x = float(x)
            waypoint.pose.y = float(y)
                
        waypoint = msg.waypoint.add()       #resend the first point
        waypoint.pose.x = float(visiting_points[0][0])
        waypoint.pose.y = float(visiting_points[0][1])
    """   
    msg = RoutingRequest()
    msg.header.module_name = 'dreamview'
    msg.header.sequence_num = 0
    
    x = visiting_points[0][0]
    y = visiting_points[0][1]
        
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x)
    waypoint.pose.y = float(y)
    
    xx = visiting_points[1][0]
    yy = visiting_points[1][1]
        
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(xx)
    waypoint.pose.y = float(yy)
    
    apollo_test.routing_writer.write(msg)
    time.sleep(20.0)
    
    
    msg2 = RoutingRequest()
    msg2.header.module_name = 'dreamview'
    msg2.header.sequence_num = 0
    
    x2 = visiting_points[2][0]
    y2 = visiting_points[2][1]
        
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x2)
    waypoint.pose.y = float(y2)
    print(msg)
    apollo_test.routing_writer.write(msg2)
    
    time.sleep(6.0)
   
    print('A routing request has been sent ..')
   
  
   
   
   
   
   
   
   
