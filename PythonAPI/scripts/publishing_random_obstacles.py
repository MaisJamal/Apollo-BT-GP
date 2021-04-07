#!/usr/bin/python
# -*- coding: utf-8 -*-

from cyber_py3 import cyber, cyber_time

import datetime
from random import *
import math
import time
import os

from genetic_algorithm import GA
#import TreeFunctions
from modules.routing.proto.routing_pb2 import RoutingRequest, \
    LaneWaypoint
from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacle, \
    PerceptionObstacles

from modules.localization.proto.localization_pb2 import LocalizationEstimate


# from modules.dreamview.proto.hmi_config import HMIAction





class ApolloFeatures:

    def __init__(self):
        cyber.init()
        self.node = cyber.Node('apollo_features')
        self.routing_writer = \
            self.node.create_writer('/apollo/routing_request',
                                    RoutingRequest)
        self.obstacle_writer = \
            self.node.create_writer('/apollo/perception/obstacles',
                                    PerceptionObstacles)

      #  self.reader_node = cyber.Node("reader")
      #  self.location_reader = self.reader_node.create_reader('/apollo/localization/pose', LocalizationEstimate, callback)

    def launch_control(self):
        os.system('cyber_launch start /apollo/modules/control/launch/control.launch &'
                  )

    def launch_routing(self):
        os.system('cyber_launch start /apollo/modules/routing/launch/routing.launch &'
                  )

    def launch_perception(self):
        os.system('cyber_launch start /apollo/modules/transform/launch/static_transform.launch &'
                  )
        os.system('cyber_launch start /apollo/modules/perception/production/launch/perception.launch &'
                  )

    def launch_prediction(self):
        os.system('cyber_launch start /apollo/modules/prediction/launch/prediction.launch &'
                  )

    def launch_rtk_localization(self):
        os.system('ldconfig -p | grep libcuda.so')
        os.system('cyber_launch start /apollo/modules/localization/launch/rtk_localization.launch &'
                  )

    def launch_planning(self):
        os.system('cyber_launch start /apollo/modules/planning/launch/planning.launch &'
                  )

    def launch_all_modules(self):
        self.launch_rtk_localization()
        self.launch_perception()
        self.launch_prediction()
        self.launch_planning()
        self.launch_control()
        self.launch_routing()

    def send_routing_request(
        self,
        x_start,
        y_start,
        x_end,
        y_end,
        ):
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
        
        
#GENES = 'abcdefghijklmnopqrstuvwxyz&/'
#Goal = '/(&(d/(&(efgb)&(hijc)))a)'


if __name__ == '__main__':
    apollo_test = ApolloFeatures()
    seq = 0
    #time.sleep(2.0)
    """
    generation = [['g',0],['df',0],['fe',0]]
    new_generation = [['g',0],['ggg',0],['sfg',0],['gllsg',0],['fllg',0]]
    repeated = False
    if generation[0] == new_generation[0]:
        print("equal")
    for i in range(len(generation)):
        repeated = False
        for individual in new_generation:
            print("individual: ",individual)
            if generation[i][0] in individual:
                repeated = True
        if not repeated:
            new_generation.append(generation[i])
    print("repeated ",repeated)
    print(list(new_generation))
    input('Press Enter to continue ...')
    """
    map_type = 'Highway4'  # NKB_cutted

    # Borregas Ave

    if map_type == 'Borregas_ave':
        routing_1_x = 587034.795623779
        routing_1_y = 4141516.708694458
        routing_2_x = 586952.61
        routing_2_y = 4141207.97
        
    elif map_type == 'test':   # NKB cutted with 3 lines
        routing_1_x = 395800.30
        routing_1_y = 6246085.90
        routing_2_x = 395979.71
        routing_2_y = 6246081.28
        
    elif map_type == 'Sunnyvale_Big_loop':     # Sunnyvale Big loop
        routing_1_x = 586869.96
        routing_1_y = 4140260.31
        routing_2_x = 587216.36
        routing_2_y = 4140363.5
        
    elif map_type == 'NKB_cutted':   # NKB_straight_line
        routing_1_x = 395960.93
        routing_1_y = 6246150.59
        routing_2_x = 395314.47
        routing_2_y = 6246161.79
        
    elif map_type == 'Highway3':     # highway3
        routing_1_x = 420123.70
        routing_1_y = 6182599.60
        routing_2_x = 420489.18
        routing_2_y = 6182618.67
        
    elif map_type == 'Highway4': # highway4 long one
        routing_1_x = 444084.19   # first lane: 444084.79  # second lane: 444084.19
        routing_1_y = 6185611.08 # first lane: 6185607.61 # second lane: 6185611.08
        routing_2_x = 444986.63  # first lane: 444996.08,second lane:444986.63 , third lane:
        routing_2_y = 6185779.9  # first lane: 6185778.55,second lane:6185779.9, third lane: ....
        
    else:
        print ('map type does not match any map..')
        routing_1_x = 587034.795623779
        routing_1_y = 4141516.708694458
        routing_2_x = 586952.61
        routing_2_y = 4141207.97



    starting_distance = 40  # starting distance between the 2 vehicles

    theta = math.atan2(routing_2_y - routing_1_y, routing_2_x
                       - routing_1_x)


    # [starting_distacne(-30,100), initial_x, initial_y, delta_width(0-5), speed, old_x, old_y]
    """
    
    front_obs_id = -1
    obstacles = []
    for i in range(ObstaclesNo):
        if i==0:
            delta_width = 0
            starting_distance =  20 #randint(20, 40)
            front_obs_id = i
            speed = randint(27, 55) / 10  
        else:
            rand_bi = randrange(0, 2)%2
            if rand_bi == 1:
                sign = 1
            else:
                sign = -1
            delta_width = 3.4 * sign #(-1,5)
            starting_distance = randint(-60, 10)
            speed = randint(27, 75) / 10  # between 2.77 m/s(10 km/h) and 5.55m/s (20 km/h)
            
        initial_x = routing_1_x + math.cos(theta) * starting_distance  # initial position of obstacle
        initial_y = routing_1_y + math.sin(theta) * starting_distance  # initial position of obstacle


        old_x = initial_x
        old_y = initial_y
        obstacles.append([
            starting_distance,
            initial_x,
            initial_y,
            delta_width,
            speed,
            old_x,
            old_y,
            ])
        print (obstacles[i])

    T = 0.1  # in seconds/ sleep time
    approx_time_keep_lane = 209 / obstacles[front_obs_id][4]  # time for keeping the lane = distance/front obstacle speed

    """
    
 
    # ######## Genetic algorithm #############
    ga = GA()

    # first_generation 
    population = []
    """
    population.append(["/(&(ceg/(uvwxz)Y)&(ikmZ)X)",0]) # switch to left first
    population.append(["/(&(cegY)&(ikmZ)X)",0]) # keep lane
    population.append(["/(&(ikmZ)&(cegY)X)",0]) # switch to right first
    population.append(["/(&(jkgvY)X)",0])
    population.append(["/(&(ceg/(knvwxz)Y)&(ikmZ)X)",0])   
    population.append(['/(&(/(&(/(dyf&(zirfdZ)X)jizY)nY)Y)hyZ)', 0])
    population.append(['/(&(r&(rpc)ewv&(g&(jkZ)g&(lrZ)X)pzZ)X)', 0])
    population.append(['/(&(/(dn/(ks&(s&(odeuiY))))Z)X)', 0])
    population.append(['/(&(ug&(cpeyuzjm/(crX)eY)nqY)Z)', 0])
    population.append(['/(&(cgnfZ)/(gfpY)rZ)', 0])
    
    population.append(["/(&(opceg/(uvwxz)Y)&(ikmZ)X)",0]) # switch to left first
    population.append(["/(&(cegY)&(ikuvmZ)X)",0]) # keep lane
    population.append(["/(&(iwrkmZ)&(cegY)X)",0]) # switch to right first
    population.append(["/(&(xyqrZ)Y)",0])
    population.append(["/(&(lfuzY)Z)",0])   
    population.append(['/(&(&(/(dyf&(z&(izrY)Z)Z)jizY)nY)hyZ)', 0])
    population.append(['/(&(rewv&(g&(jkZ)g&(lrZ)X)pzZ)X)',0])
    population.append(['/(&(zhpw/(dn/(s&(ode/(ui))Z)))Y)X)',0])
    population.append(['/(&(ug&(c&(peyuzjm)&(crX)eY)nqY)Z)', 0])
    population.append(['/(pg&(cginZ)/(gfpY)rZ)', 0])
    
    population.append(['/(h)', 54.899742603302]) 
    population.append(['/(&(ygt&(ciypeytznjmik/(c&(wX)hY)nq&(&(r)r/(f)X)Y))Z)', 17.76700210571289]) 
    population.append(['/(/(Z)&(X))', 27.67224561251127])
    population.append(['/(&(ygt&(ciypeytzdjkmhY)nk&(&(r)mY)h)Z)', 17.164169549942017])
    population.append(['/(&(ygt&(ciypeytznjmik/(c&(wX)hY)nq&(&(r)/(un)/(f)X)Y))Z)', 29.030371656784645])   
    population.append(['/(/(j&(/(r)Z)))', 54.8995406627655])
    population.append(['/(&(ygt&(ciypeytznjmik/(c&(wX)hY)nq&(&(r)/(un)/(k)X)Y))Z)', 22.54704039830428])
    population.append(['/(qygt&(ciypeytzdjmkmhY)nk&(&(r)mY)hZ)', 54.89970016479492])
    population.append(['/(&(yygt&(cm/(Z)eyt/(ydjmi)kkY)nq/(Y))Z)', 42.17167925726774])
    population.append(['/(&(ygt&(ciypeytzdjmikmh)nk&(&(r)mY)h)Z)', 42.372712402457026])
    
    population.append(['/(&(ygt&(ciypeytzdjmkmhk)nk&(&(r)mY)h)Z)', 54.899656534194946]) 
    population.append(['/(&(yyht&(cm/(Z)eyt/(ydjmi)kkY)nq/(Y))Z)', 54.899699687957764]) 
    population.append(['/(/(/(j&(r))))X', 47.49576078928434]) 
    population.append(['/(&(ygt&(ciypeytznjmik/(c&(wX)hY)nq&(&(r)/(un)/(f)X)Y)))', 54.89967107772827])
    population.append(['/(&(ygt&(ciypeytz/(Y)jmik/(c&(wX)hY)nq&(&(Y)/(un)/(k)X)Y))Z)', 32.73250007170897])   
    population.append(['/(/(j&(rZ)))', 54.89964580535889])
    population.append(['/(&(ygt&(ciypeytzdjmikmnh)nk&(&(r)mY)h)Z)', 54.899659156799316])
    population.append(['/(&(ygt&(ciypeytzdjm/(kmhY))nk&(&(r)mY)h)Z)', 17.763444423675537])
    population.append(['/(&(ygt&(ciyptytzdjmikmh)nk&(&(r)mY)h)Z)', 17.767680883407593])
    population.append(['/(&(ygt&(ciypeytzdjmkmhY)nok&(&(r)mY)h)Z)', 54.899763345718384])
    """
    
    generation_number = 1 
    # randomly generate indiviuals and evaluate them
    i = 0
    while i < (ga.population_size):
        individual = ga.create_individual()
        while(not ga.IsValidTree(individual)):
            print("invalid tree.. ")
            individual = ga.create_individual()
      #  individual[1] = ga.fittness_eval(individual[0]) 
        population.append([individual,0]) 
        i = i+1 
     
    #print(population)
    #input('Press Enter to continue ...')
    """ first population evaluation , no need for now 
    for tree in population:#first_generation:
        time.sleep(1)
        open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
        f = open("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/TreeStruct.txt", "w")
        f.write(tree[0])
        f.close()
        print("evaluating tree: ",tree[0])
                
        for i in range(ObstaclesNo):
            obstacles[i][5] = obstacles[i][1]
            obstacles[i][6] = obstacles[i][2]
        
        apollo_test.send_routing_request(routing_1_x, routing_1_y,
            routing_2_x, routing_2_y)

        print ('A routing request has been sent ..')

        start_time = cyber_time.Time.now().to_sec()
        stop = False
        while not cyber.is_shutdown() and not stop:

         #   print('publishing obstacles')

            msg = PerceptionObstacles()
            msg.header.module_name = 'perception_obstacle'
            msg.header.sequence_num = seq
            msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
            msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
            seq = seq + 1
    
            for i in range(ObstaclesNo):

                obstacle = msg.perception_obstacle.add()
    
                obstacle.id = i
                obstacle.theta = theta  # in radian
                obstacle.position.x = obstacles[i][5] + math.cos(theta) \
                    * (T * obstacles[i][4]) - math.sin(theta) \
                    * obstacles[i][3]  # delta_s = T* speed
                obstacle.position.y = obstacles[i][6] + math.sin(theta) \
                    * (T * obstacles[i][4]) + math.cos(theta) \
                    * obstacles[i][3]
                obstacle.position.z = 0
                obstacles[i][5] = obstacles[i][5] + math.cos(theta) * (T
                        * obstacles[i][4])
                obstacles[i][6] = obstacles[i][6] + math.sin(theta) * (T
                        * obstacles[i][4])

                obstacle.velocity.x = math.cos(theta) * obstacles[i][4]
                obstacle.velocity.y = math.sin(theta) * obstacles[i][4]
                obstacle.velocity.z = 0

                obstacle.length = 4.565
                obstacle.width = 2.082
                obstacle.height = 1.35

                obstacle.tracking_time = cyber_time.Time.now().to_sec() \
                    - start_time

                obstacle.type = 5
                obstacle.timestamp = time.time()

              #  print(obstacles[i])

            time.sleep(T)
            apollo_test.obstacle_writer.write(msg)
            
            end_time = cyber_time.Time.now().to_sec()
            delta_time = end_time - start_time # in seconds

      #      test_start = cyber_time.Time.now().to_sec()
            f_result = open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','r')
            line = f_result.read()
       #     test_end = cyber_time.Time.now().to_sec()
          #  print("time for opening the file and reading a line",test_end - test_start)
            
            if (line.find('Collision') != -1) :    
                tree[1] = -200 #fitness
                stop = True
                f_result.close()
                
            elif (line.find('goal') != -1) :
                print("goal is reached without collision, testing the next tree...")
                print("time for reaching the goal by cyber... ", delta_time )
                tree[1] = approx_time_keep_lane - delta_time  #fitness 
                print("fitness value is ", tree[1])
                stop = True
                f_result.close()
                
            elif (line.find('invalid') != -1) :
                print("invalid tree, execution returned false...")
                tree[1] = -700
                print("fitness value is ", tree[1])
                stop = True
                f_result.close()
                time.sleep(3)

                
            elif (delta_time> (2*approx_time_keep_lane)):
                tree[1] = -500
                print("fitness value is ", tree[1])
                stop = True
                f_result.close()
    """
    population = sorted(population, key = lambda l:l[1], reverse = True)        
  #  for tree in population:
   #     print(tree)        
            
            
    with open("/apollo/PythonAPI/scripts/Last_generation.txt", mode='a') as file:
        file.write('\nRecorded at %s.\n\n' % datetime.datetime.now())
        file.write('first population:\n')
        for individual in population:
            file.write(str(individual))
            file.write('\n')        
            
    ObstaclesNo = 6 #random.randint(4, 10)
    max_ego_speed = 11.11#40 km/h
    min_reach_goal_time = 209/max_ego_speed
    
    while(generation_number<=ga.max_generations):
        parents = []
        ######### create a scenario for evaluation #################################
        front_obs_id = -1
        obstacles = []
        for i in range(ObstaclesNo):
            if i==0:
                delta_width = 0
                starting_distance =  20 #randint(20, 40)
                front_obs_id = i
                speed = randint(27, 55) / 10  
            else:
                rand_bi = randrange(0, 2)%2
                if rand_bi == 1:
                    sign = 1
                else:
                    sign = -1
                delta_width = 3.4 * sign #(-1,5)
                starting_distance = randint(-50, 10)
                speed = randint(27, 65) / 10  # between 2.77 m/s(10 km/h) and 5.55m/s (20 km/h)
            
            initial_x = routing_1_x + math.cos(theta) * starting_distance  # initial position of obstacle
            initial_y = routing_1_y + math.sin(theta) * starting_distance  # initial position of obstacle


            old_x = initial_x
            old_y = initial_y
            obstacles.append([
                starting_distance,
                initial_x,
                initial_y,
                delta_width,
                speed,
                old_x,
                old_y,
                ])
        print(obstacles)
        
        with open("/apollo/PythonAPI/scripts/Last_generation.txt", mode='a') as file:
            file.write('\nScenario obstacles  :\n')
            for obs_data in obstacles:
                file.write(str(obs_data))
                file.write('\n') 
                
        T = 0.1  # in seconds/ sleep time
        approx_time_keep_lane = 209 / obstacles[front_obs_id][4]  # time for keeping the lane = distance/front obstacle speed
        ##########################################################################
        ################evaluation on the new scenario############################
        
        k=0
        while k < len(population):
            #time.sleep(1)
            open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
            f = open("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/TreeStruct.txt", "w")
            f.write(population[k][0])
            f.close()
            print("evaluating tree: ",population[k][0])
                
            for i in range(ObstaclesNo):
                obstacles[i][5] = obstacles[i][1]
                obstacles[i][6] = obstacles[i][2]
           # time.sleep(1)
            apollo_test.send_routing_request(routing_1_x, routing_1_y,
                routing_2_x, routing_2_y)
            
            
            #open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
            #print ('A routing request has been sent ..')

            start_time = cyber_time.Time.now().to_sec() 
            stop = False
            while not cyber.is_shutdown() and not stop:

         #   print('publishing obstacles')

                msg = PerceptionObstacles()
                msg.header.module_name = 'perception_obstacle'
                msg.header.sequence_num = seq
                msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
                seq = seq + 1
    
                for i in range(ObstaclesNo):

                    obstacle = msg.perception_obstacle.add()
    
                    obstacle.id = i
                    obstacle.theta = theta  # in radian
                    obstacle.position.x = obstacles[i][5] + math.cos(theta) \
                        * (T * obstacles[i][4]) - math.sin(theta) \
                        * obstacles[i][3]  # delta_s = T* speed
                    obstacle.position.y = obstacles[i][6] + math.sin(theta) \
                        * (T * obstacles[i][4]) + math.cos(theta) \
                        * obstacles[i][3]
                    obstacle.position.z = 0
                    obstacles[i][5] = obstacles[i][5] + math.cos(theta) * (T
                             * obstacles[i][4])
                    obstacles[i][6] = obstacles[i][6] + math.sin(theta) * (T
                             * obstacles[i][4])

                    obstacle.velocity.x = math.cos(theta) * obstacles[i][4]
                    obstacle.velocity.y = math.sin(theta) * obstacles[i][4]
                    obstacle.velocity.z = 0

                    obstacle.length = 4.565
                    obstacle.width = 2.082
                    obstacle.height = 1.35

                    obstacle.tracking_time = cyber_time.Time.now().to_sec() \
                         - start_time

                    obstacle.type = 5
                    obstacle.timestamp = time.time()

                   #print(obstacles[i])

                time.sleep(T)
                apollo_test.obstacle_writer.write(msg)
            
                end_time = cyber_time.Time.now().to_sec()
                delta_time = end_time - start_time # in seconds

               #test_start = cyber_time.Time.now().to_sec()
                f_result = open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','r')
                line = f_result.read()
               #test_end = cyber_time.Time.now().to_sec()
               #print("time for opening the file and reading a line",test_end - test_start)
            
                if (line.find('Collision') != -1) :  
                    old_fitness = population[k][1] 
                    new_fitness = -200 #fitness 
                    population[k][1]  =(old_fitness + new_fitness)/2
                    print("collision..")
                    stop = True
                    f_result.close()
                
                elif (line.find('goal') != -1) :
                    if (delta_time < min_reach_goal_time):
                        #k-=1
                        open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
                        print("reevaluating tree...")
                    else:
                        print("goal is reached without collision, testing the next tree...")
                        print("time for reaching the goal by cyber... ", delta_time )
                        old_fitness = population[k][1]  
                        new_fitness = approx_time_keep_lane - delta_time  #fitness 
                        population[k][1]  =(old_fitness + new_fitness)/2
                        print("fitness value is ", population[k][1])
                        stop = True
                    f_result.close()
                
                elif (line.find('invalid') != -1) :
                    print("invalid tree, execution returned false...")
                    old_fitness = population[k][1] 
                    new_fitness = -700
                    population[k][1]  =(old_fitness + new_fitness)/2
                    print("invalid tree..")
                    stop = True
                    f_result.close()
                    time.sleep(1.5)
 
                
                elif (delta_time> (2*approx_time_keep_lane)):
                    old_fitness = population[k][1] 
                    new_fitness = -500
                    population[k][1]  =(old_fitness + new_fitness)/2
                    print("goal never reached")
                    stop = True
                    f_result.close()
            k += 1
        
        
        ##########################################################################
        ############sorting and choosing parents, then offsprings#################
        population = sorted(population, key = lambda l:l[1], reverse = True)
      #  print("generation_number: ", generation_number , "best fittness: " , population[0][1])
      #  print(population[0])
        
        new_generation = []
        offsprings = []
        #Tournament selection technique.
        # How it works: The algorithm choose randomly two
        # individuals from the population and returns the fittest one
        for i in range(ga.population_size//2):
            selection1 = population[randrange(ga.population_size)]
            selection2 = population[randrange(ga.population_size)]
            while(selection2 == selection1):
                selection2 = population[randrange(ga.population_size)]
            if selection1[1]>selection2[1]: #compare fitness
                parents.append(selection1)
            else:
                parents.append(selection2)
        for i in range(len(parents)):
            parent1 = parents[i]          
            # 40% crossover 60% mutation
            r = randrange(100)
            if r < (ga.crossover_percent) : # do crossover
                for j in range(2):
                    parent2 = parents[randrange(len(parents))]
                    while(parent2 == parent1):
                        parent2 = parents[randrange(len(parents))]
                    #print("crossover:")
                    #print("parent1: ", parent1)
                    #print("parent2: ", parent2)
                    offspring1,offspring2 = ga.crossover(parent1[0],parent2[0])
                    while ([offspring1,-1000] in population) or ([offspring2,-1000] in population) or offspring1 == '' or offspring2 == '' or not ga.IsValidTree(offspring1) or not ga.IsValidTree(offspring2):
                        offspring1,offspring2 = ga.crossover(parent1[0],parent2[0])
                    #print("offspring1: ", offspring1)
                    #print("offspring2: ", offspring2)
                    offsprings.append([offspring1,-1000])
                    offsprings.append([offspring2,-1000])
            else: # do mutation
                for j in range(4):
                    #print("mutation: ", parent)
                    offspring = ga.mutation(parent1[0])
                 #   offspring = ga.mutation(offspring)
                    while [offspring,-1000] in population or offspring == '' or not ga.IsValidTree(offspring):
                        offspring = ga.mutation(parent1[0])
                #print("offspring: ",offspring)
                    offsprings.append([offspring,-1000])
            #input('Press Enter to continue ...')
        #print("offsprings number: ",len(offsprings))
        #for tree in offsprings:
            #print(tree)
        #############################################################################
        ##############evaluating offsprings##########################################
        k = 0
        while k < len(offsprings):
          #  time.sleep(1)
            open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
            f = open("/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/TreeStruct.txt", "w")
            f.write(offsprings[k][0])
            f.close()
            print("evaluating tree: ",offsprings[k][0])
                
            for i in range(ObstaclesNo):
                obstacles[i][5] = obstacles[i][1]
                obstacles[i][6] = obstacles[i][2]
          #  time.sleep(1)        
            apollo_test.send_routing_request(routing_1_x, routing_1_y,
                routing_2_x, routing_2_y)

            #open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
            #print ('A routing request has been sent ..')

            start_time = cyber_time.Time.now().to_sec()
            stop = False
            while not cyber.is_shutdown() and not stop:

         #   print('publishing obstacles')

                msg = PerceptionObstacles()
                msg.header.module_name = 'perception_obstacle'
                msg.header.sequence_num = seq
                msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
                msg.header.lidar_timestamp = cyber_time.Time.now().to_nsec()
                seq = seq + 1
    
                for i in range(ObstaclesNo):

                    obstacle = msg.perception_obstacle.add()
    
                    obstacle.id = i
                    obstacle.theta = theta  # in radian
                    obstacle.position.x = obstacles[i][5] + math.cos(theta) \
                        * (T * obstacles[i][4]) - math.sin(theta) \
                        * obstacles[i][3]  # delta_s = T* speed
                    obstacle.position.y = obstacles[i][6] + math.sin(theta) \
                        * (T * obstacles[i][4]) + math.cos(theta) \
                        * obstacles[i][3]
                    obstacle.position.z = 0
                    obstacles[i][5] = obstacles[i][5] + math.cos(theta) * (T
                             * obstacles[i][4])
                    obstacles[i][6] = obstacles[i][6] + math.sin(theta) * (T
                             * obstacles[i][4])

                    obstacle.velocity.x = math.cos(theta) * obstacles[i][4]
                    obstacle.velocity.y = math.sin(theta) * obstacles[i][4]
                    obstacle.velocity.z = 0

                    obstacle.length = 4.565
                    obstacle.width = 2.082
                    obstacle.height = 1.35

                    obstacle.tracking_time = cyber_time.Time.now().to_sec() \
                         - start_time

                    obstacle.type = 5
                    obstacle.timestamp = time.time()

                   #print(obstacles[i])

                time.sleep(T)
                apollo_test.obstacle_writer.write(msg)
            
                end_time = cyber_time.Time.now().to_sec()
                delta_time = end_time - start_time # in seconds

               #test_start = cyber_time.Time.now().to_sec()
                f_result = open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','r')
                line = f_result.read()
               #test_end = cyber_time.Time.now().to_sec()
               #print("time for opening the file and reading a line",test_end - test_start)
            
                if (line.find('Collision') != -1) :  
                    new_fitness = -200 #fitness 
                    offsprings[k][1]  = new_fitness
                    print("collision..")
                    stop = True
                    f_result.close()
                
                elif (line.find('goal') != -1) :
                    if (delta_time < min_reach_goal_time):
                        #k-=1
                        open('/apollo/modules/planning/tasks/deciders/behaviour_tree_decider/results.txt','w').close()
                        print("reevaluating tree...")
                    else:
                        print("goal is reached without collision, testing the next tree...")
                        print("time for reaching the goal by cyber... ", delta_time )
                        new_fitness = approx_time_keep_lane - delta_time  #fitness 
                        offsprings[k][1]  = new_fitness
                        print("fitness value is ", offsprings[k][1])
                        stop = True
                    f_result.close()
                
                elif (line.find('invalid') != -1) :
                    print("invalid tree, execution returned false...")
                    new_fitness = -700
                    offsprings[k][1]  = new_fitness
                    print("invalid tree..")
                    stop = True
                    f_result.close()
                    time.sleep(1.5)
 
                
                elif (delta_time> (2*approx_time_keep_lane)):
                    new_fitness = -500
                    offsprings[k][1]  = new_fitness
                    print("goal never reached")
                    stop = True
                    f_result.close()
            k += 1
        #print("offsprings after evaluating: ")
        #for tree in offsprings:
            #print(tree)
        #############################################################################
        ########## adding offsprings to original population (N+2N) ##################
        ########### and perform elitism and tournament selection ####################
        for m in offsprings:
            population.append(m)
        population = sorted(population, key = lambda l:l[1], reverse = True)
        
        s = (ga.elitism_percent * ga.population_size)//100
        for l in range(s):
            new_generation.append(population[l])
        s =  ga.population_size - s #left individuals
        while len(new_generation)<ga.population_size:
            selection1 = population[randrange(len(population))]
            selection2 = population[randrange(len(population))]
            while(selection2 == selection1):
                selection2 = population[randrange(len(population))]
            if selection1[1]>selection2[1]: #compare fitness
                final_selection = selection1
            else:
                final_selection = selection2
            repeated = False
            for individual in new_generation:
                if final_selection[0] in individual:
                    repeated = True
            if not repeated:
                new_generation.append(final_selection)
        #############################################################################
        population = new_generation         
        with open("/apollo/PythonAPI/scripts/Last_generation.txt", mode='a') as file:
            file.write('\nRecorded at %s.\n\n' % datetime.datetime.now())
            file.write('\nGeneration ' + str(generation_number) +' :\n')
            print("generation: ",generation_number)
            for individual in population:
                print("individual:", str(individual))
                file.write(str(individual))
                file.write('\n') 

        generation_number = generation_number + 1    
            
            
            
            
            
            
            
            
            
            
            
