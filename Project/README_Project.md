## DD2410 Instructions for running project ##

Requires ROS installed (in Linux). In this assignment, the TIAGo Robot was expected to pick an Aruco  startring from an unknown positon. The TIAGo must make use of its sensors and a prior map of the room to transport the cube safely among rooms.

A behavior tree was implemented that goes through the following main states:

### Requirements ###
1. Robot has localized itself in the apartment
2. Navigation to picking pose
3. Cube detected
4. Complete picking task 
5. Navigation with cube to second table
6. Complete placing task
7. Cube placed on table?
  1. Yes: end of mission
  2. No: go back to state 2 and repeat until success. For this, you need to respawn the cube to its original pose in case it has fallen.

