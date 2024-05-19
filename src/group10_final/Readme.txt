-----------------------------------------------------------------------
---------------------Group-10 Final Project----------------------------
-----------------------------------------------------------------------

Place the "group10_final" package inside /src folder along with packages final_project, mage_msgs, ros2_aruco

------------------------------
cd ~/final_ws
------------------------------

Build the package
-----------------------------
colcon build
source install/setup.bash
-----------------------------


Launch Gazebo and Rviz
---------------------------------------------------
ros2 launch final_project final_project.launch.py
---------------------------------------------------


In the 2nd terminal->Go to the workspace folder
-----------------------------
source install/setup.bash
-----------------------------


(Preferred)
Launch the node(aruco_subscriber) using Launch file (way_points.yaml)
---------------------------------------------
ros2 launch group10_final node.launch.py
----------------------------------------------

[OR]

The node can also be run using ros2 run command (way_points.yaml not included)
----------------------------------------------
ros2 run group10_final aruco_subs
----------------------------------------------

After starting the node, it waits for 5 Seconds beforing implementing everything

Important Note:
1) Sometimes the robot does not spawn on rviz when using ros2 launch. In that case, stop and relaunch the node until it appears in Rviz
2) If the robot appears in Rviz but does not move or the log says Goal failed,
	a)Stop the simulation
	b)Rebuild all packages (colcon build)
	c)Start Gazebo and Rviz
	d)Start the node using ros2 launch










