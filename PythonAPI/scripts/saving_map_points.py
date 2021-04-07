#!/usr/bin/env python3

from cyber_py3 import cyber, cyber_time

import math
import time
import os


from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles

from modules.localization.proto.localization_pb2 import LocalizationEstimate 

#from modules.dreamview.proto.hmi_config import HMIAction
"""
vehicle_pos_x = 587028.16
vehicle_pos_y = 4141491.93
vehicle_pos_z = 0
"""

file = open('PythonAPI/scripts/map_points.txt', 'a')

def callback(data):
  vehicle_pos_x = data.pose.position.x
  vehicle_pos_y = data.pose.position.y
 #file.write("x: "+ str(vehicle_pos_x)+" y: "+ str(vehicle_pos_y)+"\n")
  file.write( str(vehicle_pos_x)+" "+ str(vehicle_pos_y)+"\n")
 # file.close()
  #print(" hello, something is recieved")
  #  global vehicle_pos_x
  #  global vehicle_pos_y
  #  global vehicle_pos_z
    
  #  vehicle_pos_x = data.pose.position.x
  #  vehicle_pos_y = data.pose.position.y
   # vehicle_pos_z = data.pose.position.z


class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
      #  self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
     #   self.obstacle_writer = self.node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
        
        self.reader_node = cyber.Node("reader")
        self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)
    

        

if __name__ == '__main__':
    apollo_test = ApolloFeatures()
    seq=0
  #  time.sleep(2.0)
    
    
  
    
    
    
    
    apollo_test.reader_node.spin()

   # time.sleep(1)
  #  start_x = vehicle_pos_x
    
    
    #while not cyber.is_shutdown():

   
   
   
   
   
   
   
   
