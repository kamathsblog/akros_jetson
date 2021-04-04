# akros_jetson

### Description
ROS launch files and utilities for the AKROS robots based on the Jetson Nano (Jetracer2, Jetbot2). This node also contains the high-level control script, that switches between different driving modes and publishes the messages that the robot needs to drive. The following functionalities are implemented
* control_mode_switcher.py
  * Subscribes to the mode value from the joystick, the twist message from the akros_joystick package and a twist message from another node providing autonomous functionality. Similarly, for the loader attachment, this node subscribes to the loader up/down value from the joystick, from the autonomous mode and based on the mode value, a switched output is provided.
  * Based on the mode, the output twist message is either the teleop twist command from akros_joystick, the twist message from the autonomous node or a combination of both (semi-autonomous: linear velocity via teleop, angular via autonomous node). Similarly, for the loader, the output is either the up/down commands from the joystick (for the teleop and semi-autonomous mode) or from the auto mode for mode 3.
* Launch files
  * The 'launch' directory contains the launch files to drive the AKROS Ackermann and Differential platforms
  * Each launch files launches all the nodes required to drive the particular robot
  * The robot_upstart package is then used to create a service that launches the selected launch file at startup
* Gazebo files
  * The 'gazebo' directory contains the install script and the launch files for a Gazebo simulation of the NVidia Jetbot (TODO: this needs to be updated for the Jetbot2)
* CAD files
  * The 'cad' directory contains the STL/DXF files to 3D print and laser cut the custom chassis parts for two iterations of the Jetracer


### Subscribers
The node subscribes to the following topics:
* `/mode`: Selected driving mode from the joystick node
* `/teleop/cmd_vel`: Twist message received from the joystick node
* `/auto/cmd_vel`: Twist message received from the autonomy node 
* `/teleop/loader`: Loader up/down message (Int8) received from the autonomy node
* `/auto/loader`: Loader up/down message (Int8) received from the autonomy node


### Publishers
The node publishes the `/switch_node/cmd_vel` topic that is a combination of the subscribed twist messages and dependent on the selected mode. For the loader attachment, this node also publishes the `/switch_node/loader` topic that is either the teleop loader message or the auto message based on the mode.

### Launch
The following launch files provided:
* drive_jetbot2.launch: `roslaunch akros_jetson drive_jetbot2.launch`
* drive_jetracer2.launch: `roslaunch akros_jetson drive_jetracer2.launch`

