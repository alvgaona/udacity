# Write-up

Firstly, SLAM was performed by using the following packages.

- `my_robot`: Custom package that calls Gazebo and creates the environment in which
the robot will be deployed. It contains a [world][my-office.world] file called by a [launch file][world.launch].
- `turtlebot_gazebo`: Package used to launch `gmapping_demo.launch` to perform the mapping.
- `turtlebot_rviz_launchers`: Package used to launch `view_navigation.launch`.
- `turtlebot_teleop`: Package used to call `keyboard_teleop.launch` to operate the turtlebot robot.

The script [test_slam.sh] is responsible for calling those packages, each of them in one `xterm` terminal.
After the nodes have been launched, by operating the robot map will be visualized in Rviz.

The second part consists of localization and navigation. 
By using _Adaptive Montecarlo Localization_ (AMCL) the robot will localized itself and will be able
to navigate across the room using the ROS Navigation stack, which is based on the Dijkstra's,
a variant of the Uniform Cost Search algorithm, to plan our robot trajectory from start to goal position.

The script named [test_navigation.sh] will be launching different nodes to peform the localization and navigation.

- `my_robot`: Same package used above to launch `amcl_with_map.launch` file and the environment around the robot.
- `turtlebot_rviz_launchers`: Package used to launch `view_navigation.launch`.

Once the `xterm` terminals and Gazebo is opened, using Rviz you can send a goal to the robot and it will be able
to reach it using the algorithms named above.

Lastly, the full home service robot has been set up to accomplish a few things.

- Localization
- Navigation with markers
- Mapping
- Pick-up and drop-off goals

Two packages were created, [add_markers] and [pick_objects].
The latter will send a message to the robot so it knows its two destinations,
both pick-up point and drop-off point. 
The former will just publish the marker so it can be seen in Rviz before the pick-up
and after the drop-off.
The `add_markers` node will be responsible for checking if the virtual object has been picked up
by subscribing itself to the `/odom` topic and validating how near is to the goals.
At the same time the environment around the robot will be mapped, the robot will be localized and
it will know how to reach each goal by the previously mentioned stacks combined. 


[test_slam.sh]: ../src/scripts/test_slam.sh
[test_navigation.sh]: ../src/scripts/test_navigation.sh
[add_markers]: ../src/add_markers
[pick_objects]: ../src/pick_objects
