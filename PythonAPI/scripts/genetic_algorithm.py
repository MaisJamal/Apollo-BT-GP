#!/usr/bin/env python3


"""
This module ...
"""
import json as simplejson

from random import *

class GA:
    def __init__(self):
        """
        Load GA parameters
        """
    
        file = '/apollo/PythonAPI/scripts/GA_parameters.json'
        with open(file, 'r') as fp:
            parameters = simplejson.loads(fp.read())
        self.population_size = parameters.get('population_size')
        self.max_generations = parameters.get('max_generations')
        self.elitism_percent = parameters.get('elitism_percent')
        self.crossover_percent = parameters.get('crossover_percent')
        self.mutation_percent = parameters.get('mutation_percent')
        self.mutation_mutation_percent = parameters.get('mutation_mutation_percent')
        self.mutation_addition_percent = parameters.get('mutation_addition_percent')
        self.mutation_deletion_percent = parameters.get('mutation_deletion_percent')
        self.control_nodes_percent = parameters.get('control_nodes_percent')
        self.action_nodes_percent = parameters.get('action_nodes_percent')
        self.max_tree_depth = parameters.get('max_tree_depth')
        self.GENES = parameters.get('GENES')
        self.control_nodes = parameters.get('control_nodes')
        self.action_nodes = parameters.get('action_nodes')
      #  print("TEST: mutation_addition_percent is ")
     #   print(self.mutation_addition_percent)
    
 

    def create_individual(self):
        # some assumptions:
        # 1. a selector node is always at the start of the tree and the keep lane action is always at the end of the tree
        # 2. an action node is always  the last node of the leaves subtree
        # 3. no empty control nodes and no control nodes with one child ## add it
        individual = "/()"
        length = randint(10, 30)
        control_nodes_counter = 1
        for i in range(length): # adding random number of nodes
            add_idx = randint(2,len(individual)-1) 
            if individual[add_idx] == '(' :
                add_idx = add_idx + 1 
            rand_node_type = randrange(100)  # probability of adding a control node 
            if rand_node_type < self.control_nodes_percent :  #adding a control node
                control_nodes_counter = control_nodes_counter+1
                added_node =  self.control_nodes[ randrange(len(self.control_nodes)) ]
                if (len(individual)-add_idx)>2 : # the close of the control node is NOT at the end of the tree
                    rand_place = randint(add_idx+1, len(individual)-1)
                    while individual[rand_place-1] =='(' or individual[rand_place-1] =='&' or individual[rand_place-1] =='/':
                        rand_place = randint(add_idx+1, len(individual)-1)
                    individual = individual[0:add_idx] + added_node + '(' + individual[add_idx:rand_place] +')'+ individual[rand_place:] 
                else: # the close of the control node is at the end of the tree
                    individual = individual[0:add_idx] + added_node + '(' + individual[add_idx:] + ')'            

            else: # adding a condition node
                added_node = self.GENES[randrange(len(self.GENES))]
                individual = individual[0:add_idx] + added_node + individual[add_idx:] 
            #print(individual)
        j = randint(1,control_nodes_counter)
        add_idx = len(individual)-1
        while j != 0 :
            added_node = self.action_nodes[randrange(len(self.action_nodes))]
            while individual[add_idx] != ')' :
                add_idx = add_idx -1
            individual = individual[0:add_idx] + added_node + individual[add_idx:]         
            j= j-1
         #   elif rand_node_type < self.control_nodes_percent + self.action_nodes_percent: # adding an action node
        #            
      #              
            
                
       # individual = self.tree_fix(individual)
                
    #    print (individual)
        return individual
    
    def IsValidTree (self,individual):
        if individual.find('()') != -1 :
            return False
      #  print(individual)
        for i in range(len(individual)-3):  # delete more than action in leaves 
            if self.action_nodes.find(individual[i]) != -1 and self.action_nodes.find(individual[i+1]) != -1: 
                return False
            if self.action_nodes.find(individual[i]) != -1 and individual[i+1] != ')':
                return False
            if individual[i+1] == '(' and self.control_nodes.find(individual[i])==-1 :
                return False
              #  individual = individual[0:i+1] + individual[i+2:]
        
        if self.control_nodes.find(individual[0])==-1:
            return False
        subtrees =  1
        depth = 1
        counter = 2 
        while (subtrees > 0):
            if individual[counter] == ')' :
                subtrees = subtrees - 1
                
            if individual[counter] == '(' :
                subtrees = subtrees + 1
                if subtrees > depth: 
                    depth = subtrees 
            counter = counter + 1      
        if depth > self.max_tree_depth:
            return False
        # one more possible restriction is the ask for obs speed after only a question about the place 
        return True
    
    def fittness_eval(self,individual):
        Goal = '/(&(cegY)&(ikmZ)X)'
        fitness = 0
        if len(individual) == len(Goal) :
            shorter = len(individual)
            fitness = fitness + 15
        elif len(individual) < len(Goal):
            shorter = len(individual)
            fitness = fitness - 5* abs( len(individual) - len(Goal))
        else:
            shorter = len(Goal)
            fitness = fitness - 5* abs( len(individual) - len(Goal))
        
        counter = 0
        for i in range(shorter):
            if individual[i] == Goal[i]:
                counter = counter + 1
         
        fitness = fitness + counter

        return fitness
        
    
    def crossover(self,parent1,parent2):
        counter1 = 0
        counter2 = 0
        p1 = randint(1,len(parent1)-1)
        while parent1[p1] == '(' or parent1[p1] ==')':
            p1 = randint(1,len(parent1)-1)
        if parent1[p1] == '&' or parent1[p1] == '/' :
            subtrees =  1
            counter1 = p1 + 2 
            while (subtrees > 0):
          #      print("crossover1: ", parent1, "counter: ", counter1, " length: ", len(parent1))
                if parent1[counter1] == ')' :
                    subtrees = subtrees - 1
                if parent1[counter1] == '(' :
                    subtrees = subtrees + 1 
                counter1 = counter1 + 1
            cross1 = parent1[p1:counter1]
            before_cross1 = parent1[0:p1]
            if counter1 < len(parent1):
                after_cross1 = parent1[counter1:]
            else:
                after_cross1 = ''
        else:
            cross1 = parent1[p1]
            before_cross1 = parent1[0:p1]
            if p1 == (len(parent1)-1):
                after_cross1 = ''
            else:
                after_cross1 = parent1[p1+1:] 
          
             
        p2 = randint(1,len(parent2)-1)
        while parent2[p2] == '(' or parent2[p2] ==')':
            p2 = randint(1,len(parent2)-1)
        if parent2[p2] == '&' or parent2[p2] == '/' :
            subtrees =  1
            counter2 = p2 + 2 
            while (subtrees > 0):
  #              print("crossover parent2 length: ", len(parent2), " counter2 is ", counter2)
   #             print(parent2)
                if parent2[counter2] == ')' :
                    subtrees = subtrees - 1
                if parent2[counter2] == '(' :
                    subtrees = subtrees + 1 
                counter2 = counter2 + 1 
            cross2 = parent2[p2:counter2]
            before_cross2 = parent2[0:p2]
            if counter2 < len(parent2):
                after_cross2 = parent2[counter2:]
            else:
                after_cross2 = ''
        else:
            cross2 = parent2[p2]
            before_cross2 = parent2[0:p2]
            if p2 == (len(parent2)-1):
                after_cross2 = ''
            else:
                after_cross2 = parent2[p2+1:]       
        #print("crossover of parents:")
        #print(parent1)
        #print(parent2)
        #print("resulted two individuals:")
        
        individual1 = before_cross1 + cross2 + after_cross1
        individual2 = before_cross2 + cross1 + after_cross2
        #print(individual1)
        #print(individual2)
        return [individual1,individual2]
        

    def mutation(self,parent):
        r = randrange(100)
        if r < self.mutation_deletion_percent : # node deletion
            #print("mutation deletion:")
            #print("parent: ",parent)
            delete_idx = randint(1,len(parent)-1)
            while parent[delete_idx] == '(' or parent[delete_idx] == ')' or parent[delete_idx] == '&' or parent[delete_idx] == '/' :
                delete_idx = randrange(len(parent))
            
            parent = parent[0:delete_idx] + parent[delete_idx+1 : ]

            #print("offspring: ",parent)
        elif r < (self.mutation_deletion_percent+self.mutation_addition_percent) : # node addition
            #print("mutation addition:")
            #print("parent: ",parent)
            add_idx = randrange(2,len(parent)-1)
            if parent[add_idx] == '(' :
                add_idx = add_idx + 1  
            rand_node_type = randrange(100)  # 50% probability of adding a control node 
            if rand_node_type < self.control_nodes_percent :
                added_node =  self.control_nodes[ randrange(2) ]
            elif rand_node_type < self.control_nodes_percent + self.action_nodes_percent: # adding an action node
                added_node = self.action_nodes[randrange(len(self.action_nodes))]
                while parent[add_idx] != ')' :
                    add_idx = randint(2,len(parent)-1) 
            else: # adding a condition node
                added_node = self.GENES[randrange(len(self.GENES))]
            if added_node == '&' or added_node == '/' :
                if (len(parent)-add_idx)>2 :
                    rand_place = randint(add_idx+1, len(parent)-1)
                    while parent[rand_place-1] =='(' or parent[rand_place-1] =='&' or parent[rand_place-1] =='/':
                        rand_place = randint(add_idx+1, len(parent)-1)
                    parent = parent[0:add_idx] + added_node + '(' + parent[add_idx:rand_place] +')'+ parent[rand_place:] 
                else:
                    parent = parent[0:add_idx] + added_node + '(' + parent[add_idx:] + ')' 
            else:
                parent = parent[0:add_idx] + added_node + parent[add_idx:]
            #print("offspring: ",parent)
        else: # node mutation
            #print("mutation mutate:")
            #print("parent: ",parent)
            mutate_idx = randint(0,len(parent)-1)
            while parent[mutate_idx] == '(' or parent[mutate_idx] == ')' :
                mutate_idx = randrange(len(parent))
        
            if parent[mutate_idx] == '&' or parent[mutate_idx] == '/' :
                subtrees =  1
                counter = mutate_idx + 2 
                while (subtrees > 0):
                    if parent[counter] == ')' :
                        subtrees = subtrees - 1
                    if parent[counter] == '(' :
                        subtrees = subtrees + 1 
                    counter = counter + 1
                if counter < len(parent):
                    if not counter == (mutate_idx + 2):
                        parent = parent[0:mutate_idx] + parent[mutate_idx+2 : counter-1] + parent[counter:]
                    else:
                        parent = parent[0:mutate_idx] + parent[counter:]
                else:
                    parent = parent[0:mutate_idx] + parent[mutate_idx+2:len(parent)-1]
            else:
                parent = parent[0:mutate_idx] + parent[mutate_idx+1:]
                
            is_control_node = randrange(100)  # 50% probability of adding a control node 
            if is_control_node <50 :
                control_nodes = '&/'
                mutated_node =  control_nodes[ randrange(2) ]
            else:
                mutated_node = self.GENES[randrange(len(self.GENES)-2)]    
        
         #   rand = randrange(len(GENES))
            if mutated_node == '&' or mutated_node == '/' :
                if (len(parent)-mutate_idx)>2 :
                    rand_place = randint(mutate_idx+1, len(parent)-1)
                    while parent[rand_place-1] =='(' or parent[rand_place-1] =='&' or parent[rand_place-1] =='/':
                        rand_place = randint(mutate_idx+1, len(parent)-1)
                    parent = parent[0:mutate_idx] + mutated_node + '(' + parent[mutate_idx:rand_place] +')'+ parent[rand_place:] 
                else:
                    parent = parent[0:mutate_idx] + mutated_node + '(' + parent[mutate_idx:] + ')'  
            else:
                parent = parent[0:mutate_idx] + mutated_node + parent[mutate_idx:]
            #print("offspring: ",parent)
        while (parent.find('()') != -1):
            if(parent.find('()') < (len(parent)-2)):
                parent = parent[0:parent.find('()')-1] + parent[parent.find('()')+2:]
            else:
                parent = parent[0:parent.find('()')-1]
        for i in range(len(parent)-3):
            #print("parent: ",parent )
            if parent[i]=='/':
                subtrees =  1
                counter = i + 2 
                #print("counter: ",counter)
                IsThereAction = False
                while (subtrees > 0):
                    if self.action_nodes.find(parent[counter]) != -1 and subtrees ==1:
                        if IsThereAction :
                            parent = parent[:counter] + 'N' + parent[counter+1:]
                        else:
                            IsThereAction = True
                    elif parent[counter] == ')' :
                        subtrees = subtrees - 1
                        if IsThereAction and subtrees>0 and not parent[counter+1]== ')':
                            parent = parent[:counter] + 'N' + parent[counter+1:]
                    elif parent[counter] == '(' :
                        subtrees = subtrees + 1 
                        if IsThereAction and subtrees==1:
                            parent = parent[:counter] + 'N' + parent[counter+1:]
                        else:
                           IsThereAction = False
                    else:
                        if IsThereAction and subtrees==1:
                            parent = parent[:counter] + 'N' + parent[counter+1:]
                        else:
                           IsThereAction = False
                    counter = counter + 1
                    #print(len(parent))
                    #print("parent: ",parent)
                    #print("counter: ",counter)
                    
        #print("parent: ",parent)
        temp = parent
        parent = ''
        for i in range(len(temp)):
            if not (temp[i] =='N'):
                parent = parent + temp[i]
        #print("parent: ",parent)
        return parent
    











