#!/usr/bin/env python3

from cyber_py3 import cyber, cyber_time

import math
import keyboard
from datetime import datetime
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

def callback(data):
    global vehicle_pos_x
    global vehicle_pos_y
    global vehicle_pos_z
    
    vehicle_pos_x = data.pose.position.x
    vehicle_pos_y = data.pose.position.y
    vehicle_pos_z = data.pose.position.z

"""
class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)
        
      #  self.reader_node = cyber.Node("reader")
      #  self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)
        
        
    def launch_monitor_dreamview(self):
        os.system('./scripts/monitor.sh start')
        os.system('./scripts/dreamview.sh start')
        print('Dreamview is launched .. DREAMVIEW_URL="http://localhost:8888"')
        
        
    def launch_control(self):
        os.system('cyber_launch start /apollo/modules/control/launch/control.launch &')
    
    def launch_routing(self):
        os.system('cyber_launch start /apollo/modules/routing/launch/routing.launch &')
    
    def launch_perception(self):
        os.system('cyber_launch start /apollo/modules/transform/launch/static_transform.launch &')
        os.system('cyber_launch start /apollo/modules/perception/production/launch/perception.launch &')
    
    def launch_prediction(self):
        os.system('cyber_launch start /apollo/modules/prediction/launch/prediction.launch &')
    
    def launch_rtk_localization(self):
        os.system('ldconfig -p | grep libcuda.so')
        os.system('cyber_launch start /apollo/modules/localization/launch/rtk_localization.launch &')
    
    def launch_planning(self):
        os.system('nohup cyber_launch start /apollo/modules/planning/launch/planning.launch &')
    
    def launch_all_modules(self):
        self.launch_rtk_localization()
        self.launch_perception()
        self.launch_prediction()
        self.launch_planning()
        self.launch_control()
        self.launch_routing()
        
        
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
    	
    
    def send_routing_request(self, x_start, y_start, x_end, y_end):
        msg = RoutingRequest()
        msg.header.module_name = 'dreamview'
        msg.header.sequence_num = 0
        waypoint = msg.waypoint.add()
        waypoint.pose.x = float(x_start)
        waypoint.pose.y = float(y_start)
        waypoint = msg.waypoint.add()
        waypoint.pose.x = float(x_end)
        waypoint.pose.y = float(y_end)

        time.sleep(2.0)
        self.routing_writer.write(msg)
        
        
    def send_routing_request_from_file(self):
        # file should contain 2 colomns, one for X and the other for Y coordinates of the requested route.
        with open('PythonAPI/scripts/visiting_points.txt') as f:
            visiting_points = [[float(x) for x in line.split()] for line in f]
        
        with_priority = True
        
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

        time.sleep(2.0)
        self.routing_writer.write(msg)
        
        
    def start_recorder(self):
        now = datetime.now()
        #print("now =", now)
        # YY/mm/dd H:M:S
        dt_string = now.strftime("%Y-%m-%d.%H:%M:%S")
        print("date and time =", dt_string)	
        str = "cyber_recorder record -a -o data/bag/" + dt_string + ".record &"
        os.system(str)
        

if __name__ == '__main__':

   # while True and not keyboard.is_pressed('q'):
    #    print("hello, press q to stop ..")
   # while True:
     #   if keyboard.read_key() == "p":
     #       print("You pressed p")
     #       break    
            
    apollo_test = ApolloFeatures()
    apollo_test.launch_monitor_dreamview()
  
    time.sleep(2.0)
    input("Press Enter to continue... after chosing the vehicle, map and mode and turn on simcontrol.")
    apollo_test.launch_planning()
    time.sleep(2.0)
    apollo_test.launch_routing()
    apollo_test.launch_prediction()
    
    apollo_test.start_recorder()
    apollo_test.send_routing_request_from_file()
    input("Press Enter to continue...")
    

    
    #apollo_test.reader_node.spin()

   
    start_time = cyber_time.Time.now().to_sec()
  #  while not cyber.is_shutdown() and not keyboard.is_pressed('q'):
     
        


    apollo_test.stop_all_modules()
        
"""
   # apollo_test.send_routing_request( 596573.0,4137634.0,596562.0,4137644.0)
    time.sleep(2.0)
    apollo_test.send_routing_request( 587034.795623779,4141516.708694458,586980.0,4141311.0) # Borregas Ave/ overtaking scenario
    # before intersaction 586980.0,4141311.0
    # after intersaction 586952.0,4141210.0
    print('A routing request has been sent ..')
   
   
"""
   
   
   
   
   
   
   
   
