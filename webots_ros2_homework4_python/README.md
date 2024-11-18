# Homework 4 ROS2 - April Tag Search

This package includes the controller and launch file for Homework 4 in CS 460/560, the april tag search. The controller utilizes a wall following method to complete its search for april tags. The robot uses its right lidar readings to maneuver the room and determine if it is too close or far from the wall and will adjust accordingly. 

#### Wall Following Method
To begin, the robot will try to identify a wall on its right side or will proceed forward until it finds one, starting its wall following algorithm. If the right lidar identifies that the wall is less than its safe stop distance (which is comprised of a stop distance of 0.2 and a lider error of 0.05 for a total 0.25 meters away) it will turn and adjust left 0.1 at a rate of 0.14. If the right lidar has a reading that is greater than the previously mentioned safe stop distance plus an additional 0.2, or a 0.9 minimum reading, and the robot is not in its initial search for the wall at the start, it will turn right -0.18 at a rate of 0.14. This clause is also implemented if the robot is in its initial search for the wall and has a lidar reading less than 0.75, which is the safe stop disance plus 0.5. In the case that the robot does not hit any of these clauses, the robot feels that it is an appropriate distance to the wall and will move forward at a rate of 0.14.

#### Search for April Tags
To find the april tags, the robot will search the wall every few seconds to see if there are any tags to be found in the area. To accomplish this, the robot holds a timer of the last time it has rotated to search for the tags. Once it has been 5 seconds since it last searched and the robot has identified it is near a wall or has recently been near the wall, the robot will turn right -1.0 to search the wall. After three seconds has passed of it turning right, the robot will turn back left 1.0 for 2 seconds to return to its starting position and reset the last rotation time. This allows the entire area of the wall around the robot to be searched. With the area being searched for the april tags, the robot will resume its wall following algorithm until it is prompted to search again.

#### April Nodes

The following nodes are needed to identify the tags:
- ros2 run v4l2_camera v4l2_camera_node
- ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image_raw -r camera_info:=/camera_info --params-file  `ros2 pkg prefix apriltag_ros`/cfg/tags_36h11.yaml
- ros2 run rqt_image_view rqt_image_view
- ros2 topic echo /detections

### TO INSTALL PACKAGE FOR ASSIGNMENT - From monicadelaine/f24_robotics Github

1. Set up environment variables for ROS. Make sure to replace '/home/rpi/shared' with your own shared folder location
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Fork your own repository of Robotics-Group (using web interface)

3. Clone your fork
<pre>
git clone <your github url for this repository>
</pre>

4. Make the package (for python, it really just installs the files
<pre>
cd Robotics-Group
colcon build
</pre>

5. Set up variables to use the package you just created
<pre>
source install/setup.bash
</pre>

6. Start webots simulation with connect back to ROS in the virtual machine
<pre>
ros2 launch webots_ros2_homework4_python f23_robotics_4_launch.py
</pre>


### TEST THE CONNECTION BETWEEN ROS2 AND WEBOTS

Test the connection between webots and ROS, use a ROS based ASCII keyboard to move the simulated robot in Webots

1. Open another terminal

2. Redo the source commands (you can add to your bash to execute it automatically each time) 
<pre>
source /opt/ros/humble/setup.bash
source install/setup.bash
</pre>

3. Run the ROS-based keyboard
<pre>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
</pre>


### TO VISUALIZE LASER DATA

1. Open another terminal

2. Redo the source commands (you can add to your bash to execute it automatically each time) 
<pre>
source /opt/ros/humble/setup.bash
source install/setup.bash
</pre>

3. Run the ROS-based keyboard
<pre>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
</pre>
<pre>
  rviz2
</pre>

### RUN SAMPLE CONTROLLER

<pre>
ros2 run webots_ros2_homework4_python webots_ros2_homework4_python
</pre>

