# ros_rl_navigation_testing
A ROS package to test reinforcement learning algorithms in goal-directed navigation tasks.

## requirements
- Ubuntu 16.04 LTS
- ROS kinetic 1.12.17
- Python 2.7.12

## install
1. create a ROS workspace on your computer 
2. ```git clone``` this repository in the *src* directory of your workspace (as a ROS package)
3. run the ```catkin_make``` command in your ROS workspace

## use
1. terminal - start a ```roscore```
2. terminal - in the package *scripts* folder run ```python qserver.py``` to start the learning node (default is q-learning)
3. terminal - in the package *scripts* folder run ```python simulator.py``` to start the navigation simulator node (default is 4 states with 2 actions each and 1 reward state)

