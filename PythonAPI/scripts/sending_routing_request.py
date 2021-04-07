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
        
  
    	
    
   # def send_routing_request(self):
        

        

if __name__ == '__main__':
    apollo_test = ApolloFeatures()

    time.sleep(2.0)
    
    
    msg = RoutingRequest()
    msg.header.module_name = 'dreamview'
    msg.header.sequence_num = 0
        
    x_start =  586761.79
    y_start = 4140435.74
        
    x_1 =  586945.01
    y_1 = 4140588.99
        
    x_2 =  586759.96
    y_2 = 4140814.6
    
    
        
    x_3 =  587008.11
    y_3 = 4140790.98
        
    x_4 =  587268.49
    y_4 = 4140528.27
        
    x_5 =  587495.11
    y_5 = 4140554.19
        
    x_6 =  587646.2
    y_6 = 4140252.72
        
    x_7 =  587267.53
    y_7 = 4140730.93
        
    x_8 =  586938.03
    y_8 = 4140969.73
        
    x_9 =  587292.12
    y_9 = 4140878.48
        
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_start)
    waypoint.pose.y = float(y_start)
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_1)
    waypoint.pose.y = float(y_1)
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_2)
    waypoint.pose.y = float(y_2)
    
    apollo_test.routing_writer.write(msg)
    time.sleep(117.0)
    
    msg2 = RoutingRequest()
    msg2.header.module_name = 'dreamview'
    msg2.header.sequence_num = 1
    
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = 586846.69
    waypoint.pose.y = 4140789.55
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_2)
    waypoint.pose.y = float(y_2)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_3)
    waypoint.pose.y = float(y_3)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_4)
    waypoint.pose.y = float(y_4)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_5)
    waypoint.pose.y = float(y_5)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_6)
    waypoint.pose.y = float(y_6)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_7)
    waypoint.pose.y = float(y_7)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_8)
    waypoint.pose.y = float(y_8)
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_9)
    waypoint.pose.y = float(y_9)
        
        
    waypoint = msg2.waypoint.add()
    waypoint.pose.x = float(x_start)
    waypoint.pose.y = float(y_start)

   # time.sleep(2.0)
    apollo_test.routing_writer.write(msg2)
    
   
    print('A routing request has been sent ..')
   
  
   
   
   
   
   
   
   
