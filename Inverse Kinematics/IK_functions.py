#! /usr/bin/env python3
#import math
import numpy as np
import math
"""
    # {Badi Mirzai}
    # {badi.mirzai@gmail.com}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    h = np.sqrt((x-l0)**2 + y**2 )

    temp= (h**2 + l1**2 - l2**2)/(2*h*l1)

    q1 = math.atan2(y, x-l0) - math.acos(temp)
    
    #temp_variable = (y - l1*math.sin(q1)) / l2
    q2= math.pi - math.acos((l1**2 + l2**2 -h**2)/(2*l1*l2))
    
    q = [q1, q2, z]


    """
    Fill in your IK solution here and return the three joint values in q
    """

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
