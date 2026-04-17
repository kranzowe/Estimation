# Estimation
Estimation subsystem for ClankerCollective.

```
ros2 launch robo_rover rover_launch.py
ros2 launch rplidar_ros rplidar_a1_launch.py
ros2 run tf2 static_transform_publisher -0.0251 0.0 0.1683 0 0 0
base_link laser
ros2 launch slam_toolbox online_async_launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=ClankerCollective/src/Autonomy/rrt_planning/Course11.yaml
rviz2
ros2 run tf2_ros tf2_echo map odom
```