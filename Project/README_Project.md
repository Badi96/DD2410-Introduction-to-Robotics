## DD2410 Project information ##

The goal of this project was to implement a mission planner for [TIAGo](https://pal-robotics.com/robots/tiago/) to execute a set of tasks.
The TIAGo Robot was expected to pick an Aruco cube startring from an unknown positon and then move it to another table. The TIAGo must make use of its sensors and a prior map of the room to transport the cube safely among rooms. Furthermore, the robot was expected to be able to be "kidnapped" manually by an user and be able to start over in the task in such a way to eventually suceed with the mission. 


Simulation was done in [Gazebo](http://gazebosim.org/) of a of a [TIAGo](https://pal-robotics.com/robots/tiago/) robot in an apartment.

A behavior tree was implemented that goes through the following main states:

### Requirements ###
1. Robot has localized itself in the apartment
2. Navigation to picking pose
3. Cube detected
4. Complete picking task 
5. Navigation with cube to second table
6. Complete placing task
7. Cube placed on table?
⋅⋅⋅7.1. Yes: end of mission
⋅⋅⋅7.2. No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.

Obs 1: At any time during the navigation, a bad-intentioned TA might kidnap your robot again. Your behavior tree must be able to detect this and react to it so that the robot always knows its true position. Kidnap the robot yourself during your development to test your solution (the robot can be moved in Gazebo manually).

Obs 2: The robot uses a particle filter for localization. Use the distribution of the particles to know when the filter has converged. Other solutions will not be accepted.

## Install ##
 # Download the repository:

```terminal
cd ~/catkin_ws/src/
git clone https://github.com/ignaciotb/robi_final_project.git
```

# Build the project:

```terminal
cd ~/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash
```


# Launch simulation:

```terminal
# Launch Gazebo and RViZ
roslaunch robotics_project gazebo_project.launch

# Deploy the system and start the simulation
roslaunch robotics_project launch_project.launch
```
