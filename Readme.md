# Kinematic Model of the ROSBot XL robot
Package to demonstrate simple kinematic motions for ROSBot XL robot   

## Wheel Velocity Publisher
### Creat `wheel_velocities_publisher` Package

```bash.sh
ros2 pkg create wheel_velocities_publisher --build-type ament_cmake --dependencies rclcpp std_msgs
```

### Building package
```bash.sh
cd ~/ros2_ws
colcon build
```

### Starting the Simulation
**Requires dependent packages from `theConstruct` workspace
```bash.sh
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_xl_gazebo simulation.launch.py
```

### Initiate Basic Motion Sequence
```bash.sh
source ~/ros2_ws/install/setup.bash
ros2 run wheel_velocities_publisher wheel_velocities_publisher
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

## Kinematic Model 
### Creat `wheel_velocities_publisher` Package

```bash.sh
ros2 pkg create kinematic_model --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs
```

