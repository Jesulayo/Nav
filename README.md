NAME: SOLA AJIDE

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
   - ```roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small```.
   - In another terminal, navigate to '''uol_cmp9767m_tutorial/scripts/''' and run python perception.py. This scans the robot environment and detects the available freespace.
   - In another terminal, navigate to '''uol_cmp9767m_tutorial/scripts/''' and run python movement.py. This starts the navigation of the robot.
4. Alternatively, you can visualize the laser scan, pointstamped on RVIZ by listening to:
   - '''/thorvald_001/front_scan''' for the original laser scans
   - '''/thorvald_001/filtered_front_scan''' for the filtered laser scans
   - '''/free_space_center_point''' for the PointStamped (free space center point)
