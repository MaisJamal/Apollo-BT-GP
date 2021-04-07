#!/usr/bin/env python3

from cyber_py3 import cyber, cyber_time
import time
import os
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint


class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node("apollo_features")
        self.routing_writer = self.node.create_writer('/apollo/routing_request', RoutingRequest)

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
        os.system('cyber_launch start /apollo/modules/planning/launch/planning.launch &')
    
    def launch_all_modules(self):
        self.launch_rtk_localization()
        self.launch_perception()
        self.launch_prediction()
        self.launch_planning()
        self.launch_control()
        self.launch_routing()
    
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

if __name__ == '__main__':
    apollo_test = ApolloFeatures()
   # apollo_test.launch_all_modules()
   # apollo_test.send_routing_request( 596573.0,4137634.0,596562.0,4137644.0)
    time.sleep(2.0)
    apollo_test.send_routing_request( 587034.795623779,4141516.708694458,586980.0,4141311.0)
    #587012.0,4141432.0) # Borregas Ave/ overtaking scenario
      # before intersaction 586980.0,4141311.0
    # after intersaction 586952.0,4141210.0
    print('A routing request has been sent ..')
   
   
   
   
   
   
   
   
   
   
   
