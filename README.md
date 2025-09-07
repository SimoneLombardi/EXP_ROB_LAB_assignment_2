Experimental Robot Laboratory - Secondo assignment
==================
Simone Lombardi - 6119159

The mission for this assignment was to use plansys2 and nav2 pakages to make a robot navigate an environment, recognizing some aruco markers
and go to the lowest id marker.
The four waypoints where the marker are located are:


WP 0: x = -7.0, y =-1.5

WP 1: x = -3.0, y = -8.0

WP 2: x = 6.0, y = 2.0

WP 3: x = 7.0, y = -5.0

## How to Run the code
Running the code is separated in two parts: the navigation stack and the planning.
# Simulation and navigation
For the navigation stak and the simulation, in three different terminal run the command

```
ros2 launch robot_urdf gazebo.launch.py
```
for the slam system use:
```
ros2 launch slam_toolbox online_sync_launch.py 
```
and finally for the navigation:
```
ros2 launch nav2_bringup navigation_launch.py 

```
Remember: I modified the config file used for these last two launch file, use the package found in this repo or copy the correct config file in your packages.

# Planning
The first part is the planning domain, to run that use the command: 

```
ros2 launch erl_ass2_pkg planning_domain.launch.py 
```

and to run all the action nodes and loading the problem file use:
```
ros2 launch erl_ass2_pkg planning_problem.launch.py
```
you will find all the action nodes can create some output clogging, you can comment out the nodes from the launch file and run them in a separate terminal using:
```
ros2 run erl_ass2_pkg <..action_node_name..>
```
the names are: move_cmd, search_cmd, find_marker_cmd, end_mission_cmd

Lastly use this command to start the interactive terminal:
```
ros2 run plansys2_terminal plansys2_terminal
```
Than use these command to generate the plan and start the simulation:
```
get plan
run
```
Once the plan is generated type "run" on this terminal

## Action node description
### Move Node ###
This node is responsible for moveing the robot in the environment, it is subscribed to the `/odom` topic and it has a client for the `/navigate_to_pose` action server.
After reciveing the name of the way point, the node sends the corresponding goal to the action server.

### Search Node ###
This node is used to detect the marker in the environment, it is subscribed to the `/aruco_marker` topic. When the robot is in the correct spot this node starts publishing on the  `/cmd_vel` topic, to spin the robot at a constant 
velocity. When a marker is detected the node stores the id and current position in a map, after it uses that map to publish on a custom topic (`/lowest_marker`) the information of the lowest id marker currently visited.

### End Mission Node ###
This node is subscribed to the custom topic `/lowest_marker`, every time it recives a message it check if the marker_id is lower than the last. When all the waypoints have been visited the node uses the stored information to send a 
goal to the `/navigate_to_pose` action server. Reaching the lowest id marker in the environment.

## Problems encountered ##
The problems I encoutered were primarely rooted in the computational cost of the aruco node search, I had to lower the resolution and refresh rate of the camera and laser to be able to effectively run the simulation (the sensor parameters can be found in the robot_urdf package, under robot5.gazebo file).
And lastly the `End mission node`, was not correctly recognized from the planning server. But nontheless it works correctly. This creates a error message for the planner, that thinks that the plan failed where instead the robot is behaving correctly.
