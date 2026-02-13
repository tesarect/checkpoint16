# Kinematic Model of the ROSBot XL robot
Package to demonstrate simple kinematic motions and a pattern path that looks like `8` for ROSBot XL robot   

## Starting the Simulation
**Requires dependent packages from `theConstruct` workspace
```bash.sh
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```
## Wheel Velocity Publisher
### Initiate Basic Motion Sequence
A simple motion sequence like
- Move Forward & Backward
- Move Sideways (Left and Right)
- Turn (Clockwise and Counte-Clockwise)

will be executed, each for 3 seconds. 
Launch the Simulation first and then launch `wheel_velocities_publisher`
```bash.sh
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash && ros2 run wheel_velocities_publisher wheel_velocities_publisher
```
expected output:
```
[INFO] [1770732865.508641167] [wheel_velocities_publisher_node]: Initialized wheel velocities publisher node
[INFO] [1770732865.508736224] [wheel_velocities_publisher_node]: Move forward
[INFO] [1770732868.508842173] [wheel_velocities_publisher_node]: Move backward
[INFO] [1770732871.509031467] [wheel_velocities_publisher_node]: Move sideways-left
[INFO] [1770732874.509452441] [wheel_velocities_publisher_node]: Move sideways-right
[INFO] [1770732877.509621586] [wheel_velocities_publisher_node]: Turm clockwise
[INFO] [1770732880.509803681] [wheel_velocities_publisher_node]: Turn counter-clockwise
[INFO] [1770732883.509993161] [wheel_velocities_publisher_node]: Stop
```
### Topics confirmation
The wheel speed for individual wheels are being published under the topic `/wheel_speed`.
```bash.sh
source ~/ros2_ws/install/setup.bash
ros2 topic echo /wheel_speed
```

## Motion wrt Absolute Frame
### Initiate the pattern path `8`
Here the ROSbot XL will execute a bunch of waypoints that replicates a pattern `8` (as shown in the below image)
![Expected result](resource/expected_eight_trajectory_waypoints.png)
Launch the Simulation first and then launch `eight_trajectory`
```bash.sh
source ~/ros2_ws/install/setup.bash && ros2 launch eight_trajectory eight_trajectory.launch.py
```
### Obtained results
![waypoint trajectory](resource/eight_trajectory_path.png)

## Package creation and dependencies

```bash.sh
ros2 pkg create wheel_velocities_publisher --build-type ament_cmake --dependencies rclcpp std_msgs
ros2 pkg create kinematic_model --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs
ros2 pkg create eight_trajectory --build-type ament_cmake --dependencies rclcpp std_msgs nav_msgs tf2 tf2_ros
```
