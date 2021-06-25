
#!/usr/bin/env python3

"""
    # {Badi Mirzai}
    # {badi.mirzai@bmail.com}
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space 
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius # radius of the robot
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        # use later to get all values in the grid in between two coordiates.
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx
        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        # pose is the current position of the robot.
        # scan.angle_increment
        angle = scan.angle_min # start from the minimum angle.
        #print(pose)
        #exit()     
        #for scans in range:
        #alpha_robot = atan2(pose[1], pose[0]) # angle of the robots position with respect to origo.
        #angle_bandwidth = scan.angle_max - scan.angle_min
        #while angle <= scan.angle_max and angle >= scan.angle_min:
        
        
        x_coordinate_list = []
        y_coordinate_list = []

        
       

        for value in scan.ranges: 
            
            if value >= scan.range_max:
                angle += scan.angle_increment
                continue
            elif value <= scan.range_min:
                angle += scan.angle_increment
                continue
            else:
                delta_x = value*cos(angle + robot_yaw)
                delta_y = value*sin(angle + robot_yaw)
                resulting_x = int((pose.pose.position.x  + delta_x - origin.position.y)/resolution) 
                # position of endScan
                resulting_y = int(( pose.pose.position.y + delta_y - origin.position.y)/resolution )
                #self.add_to_map(grid_map, resulting_x , resulting_y, self.occupied_space) # fill the free space before the occupied space.  
                #end_point = [resulting_x-1, resulting_y-1]
                x_coordinate_list.append(resulting_x)
                y_coordinate_list.append(resulting_y)
                current_position = [int((pose.pose.position.x -origin.position.x)/resolution) , int((pose.pose.position.y - origin.position.y)/resolution)]
                free_space_in_between = self.raytrace(current_position,[resulting_x, resulting_y])
                #free_space_in_between = free_space_in_between[::-1] #  for row order
                for space in free_space_in_between:
                    #if self.is_in_bounds(grid_map, space[0], space[1]) == True:
                    self.add_to_map(grid_map, (space[0]), (space[1]), self.free_space)
                   
                angle += scan.angle_increment
        for i in range(len(y_coordinate_list)):
            self.add_to_map(grid_map, x_coordinate_list[i] , y_coordinate_list[i], self.occupied_space) # fill the free space before the occupied space.  
        
        
       
         
        # Only get the part that has been updated
        update = OccupancyGridUpdate() # DO NOT CHANGE!
        
        # The minimum x index in 'grid_map' that has been updated
        update.x = min(x_coordinate_list)
        # The minimum y index in 'grid_map' that has been updated
        update.y = min(y_coordinate_list)
        # Maximum x index - minimum x index + 1
        update.width = max(x_coordinate_list) - update.x + 1
        # Maximum y index - minimum y index + 1
        update.height = max(y_coordinate_list)- update.y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for y in range(update.y + update.height, update.y, -1 ):
            for x in range(update.x, update.x+ update.width):
                update.data.append(grid_map[x,y])


        """Updates the grid_map with the data from the laser scan and the pose.
    

        For E:  
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """



        """
        Fill in your solution here
        """


        """
        For C only!
        Fill in the update correctly below.
        """ 
       
        

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        for y in range(grid_map.get_height()):
            for x in range(grid_map.get_width()):
                if grid_map[x,y] == self.occupied_space:
                    for y_near in range(-5,6):
                        for x_near in range(-5,6):
                            if grid_map[x-x_near, y - y_near] != self.occupied_space:
                                if sqrt(x_near**2 + y_near**2) <= self.radius:
                                    if self.is_in_bounds(grid_map, x-x_near, y-y_near):
                                        self.add_to_map(grid_map, x-x_near , y-y_near, self.c_space)
            
        
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """

        
        # Return the inflated map
        return grid_map