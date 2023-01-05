NAME: JESULAYOMI OPEYEMI AJAYI

ID: 25948703

# RobotProgramming
This is a software system for an agricultural robot that works in a (simulated) bacchus vineyard and is used to estimate the crop yield of the field. The robot navigates autonomously around the vineyard and count all the grape bunches present. WayPoints (Nodes) are provided at specific locations to enable the robot move around. The robot is also equipped with lidars and camera sensors for obstacle avoidance and objection detectioin respectively.

# Update and Install Dependencies
Here is an automated way to install all dependencies:
1. Complete step 1 of ```Running the Simulation```
2. ```cd ~/catkin_ws/src/RobotProgramming/```
3. ```./install_dependencies.sh```

Alternatively:
1. ```sudo apt-get update && sudo apt-get upgrade```
2. ```sudo apt-get install ros-melodic-uol-cmp9767m-base ros-melodic-desktop```
3. run ```source /opt/ros/melodic/setup.bash``` on every terminal you want to use
4. ```sudo apt install python-pip```
5. ```pip install numpy``` to install numpy
6. ```pip install -U scikit-learn``` to install scikit-learn
7.  ```
    sudo apt-get install \
        ros-melodic-opencv-apps \
        ros-melodic-topological-utils \
        ros-melodic-topological-navigation \
        ros-melodic-topological-navigation-msgs \
        ros-melodic-strands-navigation \
        ros-melodic-image-geometry
     ```
# Running the Simulation
1. Create a catkin_ws:
   - ```mkdir -p ~/catkin_ws/src```
   - ```cd ~/catkin_ws/```
   - ```catkin_make```
   - ```source devel/setup.bash```
   - clone/copy this repository into ```~/catkin_ws/src```
2.  Create a folder (named ```mongodb```) in your user home directory. This folder will store all database files required to run the topological map.
Note: This is required only once
3. Launch the simulation setup:
   - ```roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small```. If it doesn't work, you may want to make sure that all simulator instances  have been terminated: ```killall -9 gzserver```
   - ```roslaunch uol_cmp9767m_tutorial topo_nav.launch```
   - ```rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/robotnavigation.yaml```
   - In another terminal run ```rosrun uol_cmp9767m_tutorial set_topo_nav_goal.py``` to move the robot. This starts the navigation of the robot and starts counting when the robot gets to WayPoint0
4. Alternatively,
   - you can visualize the topologican map by opening the topological map visualisation config for RVIZ in ```uol_cmp9767m_tutorial/config/topo_nav.rviz```
   - click the green arrows at the nodes seen in RVIZ to send topological_navigation goals to the robot
   - in a new terminal ```run rosrun uol_cmp9767m_tutorial countgrapes.py``` to count the grape bunches
 
Link to repo: https://github.com/Jesulayo/RobotProgramming.git
