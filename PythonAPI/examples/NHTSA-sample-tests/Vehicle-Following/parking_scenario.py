#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

#  for a commented script

import os
import lgsvl
import sys
import time
import evaluator

MAX_EGO_SPEED = 11.18 # (40 km/h, 25 mph)
SPEED_VARIANCE = 10 # Simple Physics does not return an accurate value

TIME_LIMIT = 360 # seconds
TIME_DELAY = 3
MAX_FOLLOWING_DISTANCE = 50 # Apollo 3.5 is very cautious



sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
if sim.current_scene == "AutonomouStuff":
    sim.reset()
else:
    sim.load("AutonomouStuff")

sim.set_time_of_day(12)
# spawn EGO in the 2nd to right lane
spawns = sim.get_spawn()
egoState = lgsvl.AgentState()

forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])
 
# 
egoState.transform.position = spawns[0].position + 20 * forward + 38.0 * right
#egoState.transform.position = lgsvl.Vector(-1,0, 10)
rot = spawns[0].rotation
rot.y = 80
egoState.transform.rotation = rot
#egoState.transform = spawns[0]

#egoState = lgsvl.AgentState()
#egoState.transform = sim.get_spawn()[0]
ego =  sim.add_agent("Apollo Modular", lgsvl.AgentType.EGO, egoState)
#ego = sim.add_agent("Jaguar2015XE (Apollo 5.0)", lgsvl.AgentType.EGO, egoState)
egoX = ego.state.position.x
egoY = ego.state.position.y
egoZ = ego.state.position.z
print('egoX  ')
print(egoX)
print('egoY  ')
print(egoY)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)

print('the scenario is runing...')
t0 = time.time()
 # The EGO should start moving first
while True:
    sim.run(0.5)
