
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Badi Mirzai}
# {badi.mirzai@gmail.com}
from math import *
from dubins import *
import random
#from math import cos, sin, atan2, pi, sqrt
#import numpy as np

#from dubins import Car
#from matplotlib import pyplot as plt 
#import numpytemp_point as np
#import time

#start_time = time.time()
class Node(object):
    def __init__(self, point, parent, theta, phi):
        # Theta is heading angle
        # phi i steering 
        self.point = point
        self.parent = parent 
        self.theta = theta
        self.phi = phi


# returns distance between two points
def distance(point1,point2):
    return(sqrt( ( (point1[0]-point2[0])**2 ) + ( (point1[1]-point2[1])**2 ) ) )



def point_inside_object(car, point): # check if point collides with object
    # return True if the point is inside the object.
    if point[0] >= car.xub:
        return True
    elif (point[0] <= car.xlb):
        return True
    elif point[1] >= car.yub:
        return True
    elif point[1] <= car.ylb:
        return True
    for objekt in car.obs:
        object_point = [objekt[0], objekt[1]]
        
        if distance(object_point, point) <= objekt[2]:
            return True
    return False

    
def point_circle_collision(car, p1, p2, radius):
    "chekcs if p1 and p2 are inside radius from each other"
    dist = distance(p1, p2)
    if dist <= radius:
        return True
    else: 
        return False

def point_close_to_target(car, point):
    # if point is 1.5 meters from target, mission accomplished. 
    target_point = [car.xt, car.yt]
    if distance(point, target_point) <= 1.1: 
        return True
    else: 
        return False
 
def create_random_point(car):
    random_point = [random.uniform(0,car.xub), random.uniform(0,car.yub)]
    #print(random_point)
    #exit()
    return random_point

def calculate_phi(current_point, next_point, theta):

    resulting_comonent_1 = next_point[0]*cos(-theta)-next_point[1]*sin(-theta)-current_point[0]*cos(-theta)+current_point[1]*sin(-theta)
    resulting_comonent_2 = next_point[0]*sin(-theta) + next_point[1]*cos(-theta)- current_point[0]*sin(-theta)-current_point[1]*cos(-theta)
    phi = atan2(resulting_comonent_2, resulting_comonent_1)
    if phi > pi/4:
        phi = pi/4
    if phi < -1*pi/4:
        phi = -1*pi/4 
    return phi



def solution(car):
    #number_of_total_nodes  = int(car.xub*car.yub)
    nodes = [] # nodes that are explored. need to append Node objects!!!!
    x_coordinate, y_coordinate = car.x0, car.y0 # starting nodes
    theta = 0 # starting heading angle
    phi = 0.1 # initial steering angle
    target_point= [car.xt, car.yt]
    current_coordinates = [x_coordinate,y_coordinate]  # coordinates of the nearest node to random point
    parent_node = Node(current_coordinates, None, theta, phi)
   
    nodes.append(parent_node) # appends first Node object for initial controls.
    controls, times = [0], [0.0, 0.01]
    parent_node = nodes[0]
        
    for i in range(100000):
        goal_node = None
        random_point = create_random_point(car)

        parent_node = nodes[0] # set some node to later find who is the closets
        
        for nod in nodes:
            if distance(nod.point, random_point)<= distance(parent_node.point, random_point):
                parent_node  = nod
        
        theta = parent_node.theta
        x_coordinate = parent_node.point[0]
        y_coordinate = parent_node.point[1]
        current_coordinates = x_coordinate,y_coordinate
        many_phi = [] # list of the phi's we get from stepping in range at each loop
        
        for m in range(22): # optimize time-computation
            phi = calculate_phi(current_coordinates, random_point, theta)
            many_phi.append(phi)
            x_coordinate,y_coordinate,theta = step(car, x_coordinate, y_coordinate, theta, phi)
            if point_inside_object(car, [x_coordinate, y_coordinate]):
                continue
        
        new_point_coordinates = (x_coordinate, y_coordinate)
        if point_inside_object(car,new_point_coordinates) == True:
            continue
        #print("not collide")
        #found_nearest_node = True
        #break
        nodes.append(Node(new_point_coordinates, parent_node, theta, many_phi))
        #temp_point = [new_point[0], new_point[1]]
     
        if point_close_to_target(car, new_point_coordinates) == True:
            print("-----------------------------------------")
            print("goal found!")
            print("-----------------------------------------")
            #print(temp_point)
            #goal_found = True
            goal_node = nodes[-1]

            break
    
    current_node = nodes[-1] # our last node is the goal node
    #temp = []
    controls = []
    
    while current_node.parent!= None:
        #print(current_node.phi)
        controls = controls + current_node.phi # glue the lists together
        #times.append(times[-1]+0.01)
        #temp.append(current_node.point)
        current_node = current_node.parent
    controls.append(current_node.phi)
    #data = np.array(temp)
    #x_t, y_t = data.T
    #plt.scatter(x_t, y_t)
    #plt.show()
    #if goal_node == None:
        #print("FAIL!!")
    
    controls = controls[::-1] # reverse the order
    times = [0]

    for i in range(len(controls)):
    
        times.append(i* 0.01 + 0.01)
    #print(len(times))
    #for t in times:
    #    print(t)


    
    #print(len(times))
    #print(len(controls))
    #exit()
    #print("------%s seconds" % (time.time() - start_time))
    return controls, times