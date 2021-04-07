#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
if sim.current_scene == "BorregasAve":
    sim.reset()
else:
    sim.load("BorregasAve")

spawns = sim.get_spawn()
layer_mask = 0
layer_mask |= 1 << 0  # 0 is the layer for the road (default)

# EGO
state = lgsvl.AgentState()
ego_position = lgsvl.Vector(342.0, 0.0, -87.7)
hit = sim.raycast(ego_position, lgsvl.Vector(0, -1, 0), layer_mask)
state.transform.position = hit.point
state.transform.rotation = lgsvl.Vector(0.0, 15.0, 0.0)
forward = lgsvl.Vector(0.2, 0.0, 0.8)
state.velocity = forward*15
a = sim.add_agent("Lincoln2017MKZ (Apollo 5.0)", lgsvl.AgentType.EGO, state)

# Pedestrian
state = lgsvl.AgentState()
pedestrian_position = lgsvl.Vector(350.0, 0.0, -12)
hit = sim.raycast(pedestrian_position, lgsvl.Vector(0, -1, 0), layer_mask)
pedestrian_rotation = lgsvl.Vector(0.0, 105.0, 0.0)
state.transform.position = hit.point
state.transform.rotation = pedestrian_rotation
pedestrian = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, state)

agents = {
    a: "EGO",
    pedestrian: "Bob"
}


# Executed upon receiving collision callback -- pedestrian is expected to walk into colliding objects
def on_collision(agent1, agent2, contact):
    name1 = agents[agent1]
    name2 = agents[agent2] if agent2 is not None else "OBSTACLE"
    print("{} collided with {}".format(name1, name2))


a.on_collision(on_collision)
pedestrian.on_collision(on_collision)

# This block creates the list of waypoints that the pedestrian will follow
# Each waypoint is an position vector paired with the speed that the pedestrian will walk to
waypoints = []

trigger = None
speed = 3
hit = sim.raycast(pedestrian_position+lgsvl.Vector(7.4, 0.0, -2.2), lgsvl.Vector(0, -1, 0), layer_mask)
effector = lgsvl.TriggerEffector("TimeToCollision", {})
trigger = lgsvl.WaypointTrigger([effector])
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=trigger)
waypoints.append(wp)

hit = sim.raycast(pedestrian_position+lgsvl.Vector(12.4, 0.0, -3.4), lgsvl.Vector(0, -1, 0), layer_mask)
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=None)
waypoints.append(wp)


def on_waypoint(agent, index):
    print("waypoint {} reached".format(index))


def agents_traversed_waypoints():
    print("All agents traversed their waypoints.")
    sim.stop()


# The above function needs to be added to the list of callbacks for the pedestrian
pedestrian.on_waypoint_reached(on_waypoint)
sim.agents_traversed_waypoints(agents_traversed_waypoints)

# The pedestrian needs to be given the list of waypoints. A bool can be passed as the 2nd argument that controls
# whether or not the pedestrian loops over the waypoints (default false)
pedestrian.follow(waypoints, False)

input("Press Enter to run")

sim.run()
