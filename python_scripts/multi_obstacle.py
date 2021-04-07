import argparse
import json
import math
import struct
import sys
import time
import random
from copy import copy

from cyber_py3 import cyber, cyber_time
from modules.routing.proto.routing_pb2 import RoutingRequest, LaneWaypoint, ParkingInfo
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, PerceptionObstacles
from modules.common.proto.geometry_pb2 import PointENU

seq = 0
T = 0.5


cyber.init()
node = cyber.Node("apollo_features")
routing_writer = node.create_writer('/apollo/routing_request', RoutingRequest)
# localization_reader = localization_pb2.LocalizationEstimate()
theta = 0

vehicle = {"length": 4.565,
           "width": 2.082,
           "height": 1.35,
           "type": "VEHICLE"}

human = {"length": 0.5,
           "width": 0.5,
           "height": 1.88,
           "type": "PEDESTRIAN"
}

bicycle = {"length": 1.5,
           "width": 0.5,
           "height": 1.88,
           "type": "BICYCLE"
}


def get_seq():
    global seq
    seq = seq + 1
    return seq

def send_routing_request(x_start, y_start, x_end, y_end):
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
    routing_writer.write(msg)

def send_parking_request(x_start, y_start, x_finish, y_finish, park_id, park_x, park_y):
    def init_parking(park_id, park_x, park_y):
        info = ParkingInfo()
        info.parking_space_id = str(park_id)
        info.parking_point.x = park_x
        info.parking_point.y = park_y
        info.parking_point.z = 0.0
        return info
    seq = get_seq()
    msg = RoutingRequest()
    msg.header.sequence_num = seq
    msg.header.module_name = "dreamview"
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_start)
    waypoint.pose.y = float(y_start)
    waypoint = msg.waypoint.add()
    waypoint.pose.x = float(x_finish)
    waypoint.pose.y = float(y_finish)
    parking = msg.parking_info
    parking.CopyFrom(init_parking(park_id, park_x, park_y))
    return msg





def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def move_away(p1, p2, dist):
    for el in range(len(p2)):
        if p1[el] < p2[el]:
            p2[el] += dist
        else:
            p2[el] -= dist
    return p2

def init_perception(description):
    """
    Create perception from description
    """
    perception = PerceptionObstacle()
    perception.id = description["id"]
    perception.position.x = description["pos"][0]
    perception.position.y = description["pos"][1]
    perception.position.z = description["pos"][2]
    perception.theta = description["theta"]
    # print(description["theta"])
    perception.velocity.x = math.cos(description["theta"]) * description["speed"]
    perception.velocity.y = math.sin(description["theta"]) * description["speed"]
    perception.velocity.z = 0
    perception.length = description["length"]
    perception.width = description["width"]
    perception.height = description["height"]
    # perception.polygon_point.extend(generate_polygon(perception.position,
    #                                                  perception.theta,
    #                                                  perception.length,
    #                                                  perception.width))
    perception.tracking_time = 20  # don't need if no goal
    perception.type = PerceptionObstacle.Type.Value(description["type"])
    perception.timestamp = cyber_time.Time.now().to_sec()
    return perception


def linear_project_perception(description, prev_perception):
    """
    Get perception from linear projection of description
    """
    theta = description['theta']
    delta_s = T * description['speed']
    if prev_perception is None:
        perception = PerceptionObstacle()
    else:
        perception = prev_perception
    perception.timestamp = cyber_time.Time.now().to_sec()
    prev_point = (prev_perception.position.x, prev_perception.position.y,
                  prev_perception.position.z)
    # print(f"{description['id']} prev point is {prev_point}")
    perception.position.x = prev_point[0] + math.cos(theta) * delta_s
    perception.position.y = prev_point[1] + math.sin(theta) * delta_s
    perception.position.z = prev_point[2]
    # print(f"{description['id']} new point is {(perception.position.x, perception.position.y, perception.position.z)} with theta: {description['theta']}")
    perception.velocity.x = math.cos(theta) * description["speed"]
    perception.velocity.y = math.sin(theta) * description["speed"]
    perception.velocity.z = 0
    perception.theta = theta
    return perception


def generate_perception(perception_description, prev_perception):
    """
    Generate perception data
    """
    seq = get_seq()
    perceptions = PerceptionObstacles()
    perceptions.header.sequence_num = seq
    perceptions.header.module_name = "perception"
    perceptions.header.timestamp_sec = cyber_time.Time.now().to_sec()
    if not perception_description:
        print('no description')
        return perceptions
    if prev_perception is None:
        for description in perception_description:
            p = perceptions.perception_obstacle.add()
            p.CopyFrom(init_perception(description))
        return perceptions
    # Linear projection
    description_dict = {desc["id"]: desc for desc in perception_description}
    # print([desc['speed'] for desc in perception_description])
    for obstacle in prev_perception.perception_obstacle:
        description = description_dict[obstacle.id]
        # print(f"description got from {obstacle.id}")
        next_obstacle = linear_project_perception(description, obstacle)
        perceptions.perception_obstacle.add().CopyFrom(next_obstacle)
    return perceptions

def obstacle_chaos_gen(map_def, number_of_descriptions, insert_point):
    description = []
    deviationFromPoint = 10  # circle around chaos hall
    prev_pos = []
    for _ in range(number_of_descriptions):
        newCoords = [insert_point[i] + random.random() * deviationFromPoint for i in range(2)]
        if prev_pos:
            for point in prev_pos:
                if distance(point, newCoords) < deviationFromPoint // 2:
                    newCoords = move_away(point, newCoords, deviationFromPoint // 2)
        else:
            prev_pos.append(newCoords)
        global seq
        seq = seq + 1
        obstacle = dict(vehicle)
        obstacle.update({"id": seq,
                         "pos": [newCoords[0], newCoords[1], 0],
                         "theta": round(random.uniform(0.0, 6.28), 2),
                         "speed": round(random.random() * deviationFromPoint) + seq
                         })
        obstacle.update(map_def)
        description.append(obstacle)

    return description

def obstacle_checkers_gen(number_of_descriptions, start_point, line_width, car_distance, theta = 0.0, speed = 3):
    description = []
    seq = get_seq()
    obstacle = dict(vehicle)
    coords = [start_point[0], start_point[1]+line_width, 0]
    obstacle.update({"id": seq,
                     "pos": coords,
                     "theta": theta,
                     "speed": speed
                     })
    description.append(obstacle)
    prev_coords = copy(coords)
    for el in range(number_of_descriptions-1):
        seq = get_seq()
        if el%2 == 0:
            y = -1*line_width
        else:
            y = line_width
        newCoords = [prev_coords[0]+car_distance, prev_coords[1]+y, prev_coords[2]]
        obstacle = dict(vehicle)
        obstacle.update({"id": seq,
                         "pos": newCoords,
                         "theta": theta,
                         "speed": speed
                         })
        description.append(obstacle)
        prev_coords = copy(newCoords)
    return description

def obstacle_crossroad_gen(start_points, num_of_obstacles, car_distance, speed):
    description = []
    for start_point in start_points:
        queue_dir = 6.28 - start_point[2]
        # todo change 300 to map bounds
        for el in range(0, 300, car_distance)[:num_of_obstacles]:
            if queue_dir < start_point[2]:
                coords = [start_point[0], start_point[1] + el, 0]
            else:
                coords = [start_point[0], start_point[1] - el, 0]
            seq = get_seq()
            obstacle = dict(vehicle)
            obstacle.update({"id": seq,
                             "pos": coords,
                             "theta": start_point[2],
                             "speed": speed
                             })
            description.append(obstacle)
    return description

def obstacle_crosswalk_gen(start_point, end_point, num_of_pedestrians, speed):

    description = []
    dtheta = 0.5 # razbros napravlenij pewehodocv
    width = 4 # width of crosswalk
    #TODO make available push pedestr from each point of crosswalk
    point = start_point
    # angel = theta
    theta = math.atan2( (end_point[1]-start_point[1]), (end_point[0]-start_point[0]))
    #print(theta)
    for ped in range(0, num_of_pedestrians):
        angle = random.uniform(theta - dtheta, theta+dtheta)
        if ped > num_of_pedestrians // 2:
            point = end_point
            angle = random.uniform(3.14 + theta - dtheta, 3.14 + theta+dtheta)
        coords = [point[0], point[1], 0]
        seq = get_seq()
        obstacle = dict(human)
        obstacle.update({"id": seq,
                         "pos": coords,
                         "theta": angle,
                         "speed": speed
                         })
        description.append(obstacle)
    return description

def obstacle_overtake_gen(main_pos, dist1, dist2, speed1, speed2):
    description = []
    line_width = 2.8
    theta = math.atan2( (main_pos[3]-main_pos[1]), (main_pos[2]-main_pos[0]))
    coords = [copy(main_pos[0])+dist1, copy(main_pos[1]), 0]
    seq = get_seq()
    obstacle = dict(vehicle)
    obstacle.update({"id": seq,
                     "pos": coords,
                     "theta": theta,
                     "speed": speed1
                     })
    description.append(obstacle)
    coords = [copy(main_pos[0])+dist2, copy(main_pos[1])+line_width, 0]
    seq = get_seq()
    obstacle = dict(vehicle)
    obstacle.update({"id": seq,
                     "pos": coords,
                     "theta": theta,
                     "speed": speed2
                     })
    description.append(obstacle)
    return description

def obstacle_parking_gen(slots):
    description = []
    for _,slot in slots.items():
        seq = get_seq()
        obstacle = dict(vehicle)
        obstacle.update({"id": seq,
                         "pos": [slot["x"], slot["y"], 0],
                         "theta": slot["theta"],
                         "speed": 0
                         })
        description.append(obstacle)
    return description


def head_on_collision_mode(map_def, carX, carY, carTheta):
    """
    works only on 1 line moving, without rotating
    """
    seq = get_seq()
    description = []
    obstacle = dict(vehicle)
    obstacle.update({"id": seq,
                "pos": [carX, carY, 0],
                "theta": 3.14 + carTheta,
                "speed": random.randint(5, 7)})
    obstacle.update(map_def)
    description.append(obstacle)
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)
    # result = create_obstacle(obstacle)
    # print(f"head_on_collision {obstacle['id']} finished: {result}")

def chaos_mode(map_def, num_of_obstacles, start_point, life_cycle=300):
    """
    TODO create obstacles around car
    now it spawn cars around the goal with random theta
    """
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_chaos_gen(map_def, num_of_obstacles, start_point)
    perception = None
    interupt = 0
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)
        interupt += 1
        if interupt == life_cycle:
            break

def checkers_mode(num_of_obstacles=4, start_point=(395804.75, 6246085.41), theta = 0.0):
    """
    play checkers
    """
    # TODO move to config or get from map
    line_width = 2.8 # full road width 5.6
    car_distance = 10  # required distance between cars
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_checkers_gen(num_of_obstacles, start_point, line_width, car_distance, theta, speed=3)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)


def fsm_mode(start_point, theta):
    """
    play fast-slow-main. On upper line - fast vehicle, on lower - slow and main behind it
    """
    # TODO move to config or get from map
    line_width = 2.8  # full road width 5.6
    car_distance = 40  # required distance between cars
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = []
    # fast vehicle
    seq = get_seq()
    obstacle = dict(vehicle)
    coords = [start_point[0] - car_distance, start_point[1]+line_width, 0]
    obstacle.update({"id": seq,
                     "pos": coords,
                     "theta": theta,
                     "speed": 15
                     })
    perception_description.append(obstacle)
    # slow vehicle
    seq = get_seq()
    obstacle = dict(vehicle)
    coords = [start_point[0] + car_distance, start_point[1], 0]
    obstacle.update({"id": seq,
                     "pos": coords,
                     "theta": theta,
                     "speed": 3
                     })
    perception_description.append(obstacle)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)


def crossroad_mode(start_points):
    """
    play crossroad simple
    """
    car_distance = 30  # required distance between cars
    num_of_obstacles = 3 # amount of obstacles from each side
    cyber.init()
    node = cyber.Node("perception")
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_crossroad_gen(start_points, num_of_obstacles, car_distance,
                                                   speed=3)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)



def crosswalk_mode(start_point, end_point, amount):
    cyber.init()
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_crosswalk_gen(start_point, end_point, amount,
                                                   speed=2)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        writer.write(perception)
        time.sleep(T)


def overtake_mode(main_pos, dist1, dist2, speed1, speed2):
    """
    dist1 - distance btw main car and forward car in same line
    dist2 - distance by s in SL coords to car on the right (left) line
    speed1 - speed of the 1 obstacle car
    speed2 - speed of the 2 obstacle car
    ------------------------
    ----M---dist1--ob1-------goal
    ----|dist2--|ob2----------
    ------------------------
    """
    cyber.init()
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_overtake_gen(main_pos, dist1, dist2, speed1, speed2)
    perception = None
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        time.sleep(T)


def parking_mode(linenum, routing_1_x, routing_1_y, routing_2_x, routing_2_y):
    import json
    with open("./python_scripts/parking_places.json", "r") as fp:
        poses = json.load(fp)
    cyber.init()
    writer = node.create_writer('/apollo/perception/obstacles', PerceptionObstacles)
    perception_description = obstacle_parking_gen(poses[linenum])
    perception = None
    # perception = generate_perception(perception_description, perception)
    # writer.write(perception)
    parking_msg = None
    if linenum == "11546":
        parking_msg = send_parking_request(routing_1_x, routing_1_y, routing_2_x, routing_2_y, park_id=int(linenum), park_x=587204.310000000,
                             park_y=4141412.560000000)
    elif linenum == "11543":
        parking_msg = send_parking_request(routing_1_x, routing_1_y, routing_2_x, routing_2_y, park_id=int(linenum), park_x=587196.72, park_y=4141414.82)
    while not cyber.is_shutdown():
        perception = generate_perception(perception_description, perception)
        # print(str(perception))
        writer.write(perception)
        routing_writer.write(parking_msg)
        time.sleep(T)



def main(args):
    parser = argparse.ArgumentParser(description="create fake perception obstacles",
                                     prog="multi_obstacle.py")
    parser.add_argument(dest="maptype", nargs='?', help = 'choose map from NKB/Crosswalk/Highway/Fiveparking')
    parser.add_argument(dest="mode", nargs='?', help='choose mode from chaos/headon/checkers/fsm/crossroad/crosswalk/overtake/line1/line8')
    args = parser.parse_args(args)

    with open("./python_scripts/start_coords.json", "r") as st:
        map_def = json.load(st)[args.maptype]
        routing_1_x, routing_1_y, routing_2_x, routing_2_y = map_def[args.mode]
        if args.maptype != "Twoofpark":
            send_routing_request(routing_1_x, routing_1_y, routing_2_x, routing_2_y)
    if args.maptype == "NKB":
        print("NKB map support modes: chaos/headon/checkers/fsm/crossroad/")
        nkb_map_def = {"lx": 395771.91, "ly": 6246082.53,
                       "rx": 395993.53, "ry": 6246152.91}
        if args.mode == "chaos":
            global theta
            theta = math.atan2(routing_2_y - routing_1_y,
                               routing_2_x - routing_1_x)
            chaos_mode(nkb_map_def, num_of_obstacles=5, start_point=(395988.71, 6246083.55), life_cycle=15)
            chaos_mode(nkb_map_def, num_of_obstacles = 5, start_point = (395991.71, 6246112.55), life_cycle=15)
            chaos_mode(nkb_map_def, num_of_obstacles = 5, start_point = (395992.12, 6246131.49), life_cycle=15)
        elif args.mode == "headon":
            theta = math.atan2(routing_2_y - routing_1_y,
                               routing_2_x - routing_1_x)
            head_on_collision_mode(nkb_map_def, routing_2_x, routing_2_y, theta)
        elif args.mode == "checkers":
            checkers_mode(num_of_obstacles=5, start_point=(routing_1_x, routing_1_y), theta=6.25)
        elif args.mode == "fsm":
            fsm_mode(start_point=(routing_1_x, routing_1_y), theta=6.25)
        elif args.mode == "crossroad":
            theta = 3.14
            crossroad_mode(start_points=[(395748.22, 6246221.16, 1.57), (395744.13, 6246239.11, 4.71)])
    elif args.maptype == "Crosswalk":
        print("Crosswalk map support modes: crosswalk")
        if args.mode == "crosswalk":
            crosswalk_mode(start_point=(412967.34, 6176608.52), end_point = (412968.66, 6176630.36), amount = 8)
    elif args.maptype == "Highway":
        print("Highway map support modes: overtake")
        if args.mode == "overtake":
            main_pos = (routing_1_x, routing_1_y, routing_2_x, routing_2_y)
            overtake_mode(main_pos, dist1 = 10 , dist2 = 5, speed1 = 5, speed2 = 8)
    elif args.maptype == "Fiveparking":
        print("Fiveparking map support modes: line1/line8")
        lines = ["line1", "line8"]
        if not args.mode in lines:
            print("choose line from {0}".format(lines))
        if args.mode == "line1":
            parking_mode(args.mode, routing_1_x, routing_1_y, routing_2_x, routing_2_y)
        elif args.mode == "line8":
            parking_mode(args.mode, routing_1_x, routing_1_y, routing_2_x, routing_2_y)
    elif args.maptype == "Twoofpark":
        parking_mode(args.mode, routing_1_x, routing_1_y, routing_2_x, routing_2_y)



if __name__ == '__main__':
    main(sys.argv[1:])

