#!/usr/bin/env python3

from cyber_py3 import cyber, cyber_time

import math
import time
import os


from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint


from modules.localization.proto.localization_pb2 import LocalizationEstimate 

#from modules.dreamview.proto.hmi_config import HMIAction



priority = []
with_priority = True
idx = 2

# file should contain 2 colomns, one for X and the other for Y coordinates of the requested route.
with open('PythonAPI/scripts/visiting_points.txt') as f:
    visiting_points = [[float(x) for x in line.split()] for line in f]
    
points_number = len(visiting_points)    
   
if with_priority :
    with open('PythonAPI/scripts/routing_with_priority.txt') as h:
        line = h.readline()
        for i in line.split(): 
            if i.isdigit() == True:
                priority.append(int(i)) 
print(priority)
      
class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
        self.reader_node = cyber.Node("reader")
        self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, self.callback )
        
    def launch_monitor_dreamview(self):
        os.system('./scripts/monitor.sh start')
        os.system('./scripts/dreamview.sh start')
        print('Dreamview is launched .. DREAMVIEW_URL="http://localhost:8888"')
       
    def launch_prediction(self):
        os.system('cyber_launch start /apollo/modules/prediction/launch/prediction.launch &')
        
    def launch_routing(self):
        os.system('cyber_launch start /apollo/modules/routing/launch/routing.launch &')
        
    def launch_planning(self):
        os.system('nohup cyber_launch start /apollo/modules/planning/launch/planning.launch &')
    
    def stop_all_modules(self):
        #stop recording
        os.system('pkill -SIGINT -f record')
        #stop module prediction
        os.system('cyber_launch stop /apollo/modules/prediction/launch/prediction.launch')
        #stop module routing 
        os.system('cyber_launch stop /apollo/modules/routing/launch/routing.launch')
    	#stop module planning
        os.system('nohup cyber_launch stop modules/planning/launch/planning.launch')
        time.sleep(2.0)
        #stop monitor and dreamview
        os.system('./scripts/dreamview.sh stop')
        os.system('./scripts/monitor.sh stop')
        
        #shutdown mainboard
        os.system('kill $(pgrep mainboard)')
    #    this.stop_all_modules()
    def callback(self,data):
        global idx
        vehicle_pos_x = data.pose.position.x
        vehicle_pos_y = data.pose.position.y
        if with_priority :
          #  print(abs(vehicle_pos_x - visiting_points[priority[idx-1]][0] ) < 10)
         #   print(abs(vehicle_pos_y - visiting_points[priority[idx-1]][1] ) < 10)
            if abs(vehicle_pos_x - visiting_points[priority[idx-1]][0] ) < 5 and abs(vehicle_pos_y -visiting_points[priority[idx-1]][1] ) < 5 :
                print('has entered the if')
                msg = RoutingRequest()
                #msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                msg.header.module_name = 'dreamview'
                msg.header.sequence_num = idx-1
                xx = visiting_points[priority[idx-1]][0]
                yy = visiting_points[priority[idx-1]][1]
        
                waypoint = msg.waypoint.add()
                waypoint.pose.x = float(xx)
                waypoint.pose.y = float(yy)
                
                if idx == points_number :
                    x = visiting_points[priority[0]][0]
                    y = visiting_points[priority[0]][1]
        
                    waypoint = msg.waypoint.add()
                    waypoint.pose.x = float(x)
                    waypoint.pose.y = float(y)
                    self.routing_writer.write(msg)
                else:
                    x = visiting_points[priority[idx]][0]
                    y = visiting_points[priority[idx]][1]
            
                    waypoint = msg.waypoint.add()
                    waypoint.pose.x = float(x)
                    waypoint.pose.y = float(y)
                    idx = idx+1
                    print("location " + str(priority[idx-1]) + " is reached")
                    self.routing_writer.write(msg)
                time.sleep(2.0)
                 
        else:
            if abs(vehicle_pos_x -visiting_points[idx-1][0] ) < 5 and abs(vehicle_pos_y -visiting_points[idx-1][1] ) < 5 :
                msg = RoutingRequest()
                msg.header.module_name = 'dreamview'
                msg.header.sequence_num = 0
                
                xx = visiting_points[idx-1][0]
                yy = visiting_points[idx-1][1]
        
                waypoint = msg.waypoint.add()
                waypoint.pose.x = float(xx)
                waypoint.pose.y = float(yy)
                
                if idx == points_number :
                    x = visiting_points[0][0]
                    y = visiting_points[0][1]
        
                    waypoint = msg.waypoint.add()
                    waypoint.pose.x = float(x)
                    waypoint.pose.y = float(y)
                else:
                    x = visiting_points[idx][0]
                    y = visiting_points[idx][1]
            
                    waypoint = msg.waypoint.add()
                    waypoint.pose.x = float(x)
                    waypoint.pose.y = float(y)
                    idx = idx+1
                    print("location " + str(priority[idx-1]) + " is reached")
                self.routing_writer.write(msg) 

        



if __name__ == '__main__':
    apollo_test = ApolloFeatures()
    
    apollo_test.launch_monitor_dreamview()
  
    time.sleep(2.0)
    input("Press Enter to continue... after chosing the vehicle, map and mode and turn on simcontrol.")
    apollo_test.launch_planning()
    time.sleep(2.0)
    apollo_test.launch_routing()
    apollo_test.launch_prediction()
    
  #  apollo_test.start_recorder()

    input("Press Enter to start route...")
    


   
  #  while not cyber.is_shutdown() and not keyboard.is_pressed('q'):
     
    """
    # file should contain 2 colomns, one for X and the other for Y coordinates of the requested route.
    with open('PythonAPI/scripts/visiting_points.txt') as f:
        visiting_points = [[float(x) for x in line.split()] for line in f]
    
    points_number = len(visiting_points)    
    """

        
    msg = RoutingRequest()
    msg.header.module_name = 'dreamview'
    msg.header.sequence_num = 0
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    if with_priority :
        """
        with open('PythonAPI/scripts/routing_with_priority.txt') as h:
            line = h.readline()
            for i in line.split(): 
                if i.isdigit() == True:
                        priority.append(int(i)) 
        """
        print(priority)
        for i in range(2):
            x = visiting_points[priority[i]][0]
            y = visiting_points[priority[i]][1]
        
            waypoint = msg.waypoint.add()
            waypoint.pose.x = float(x)
            waypoint.pose.y = float(y)
    else:

        for i in range(2):
            x = visiting_points[i][0]
            y = visiting_points[i][1]
        
            waypoint = msg.waypoint.add()
            waypoint.pose.x = float(x)
            waypoint.pose.y = float(y)
    time.sleep(2.0)       
    apollo_test.routing_writer.write(msg)    
    
    apollo_test.reader_node.spin()
        
        
     #   waypoint = msg.waypoint.add()       #resend the first point
     #   waypoint.pose.x = float(visiting_points[0][0])
     #   waypoint.pose.y = float(visiting_points[0][1])
    
  
    
    
    
    
   # apollo_test.reader_node.spin()

   # time.sleep(1)
  #  start_x = vehicle_pos_x
    
    
    #while not cyber.is_shutdown():

   
   
   
   
   
   
   
   
