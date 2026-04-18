# How to run localization in slam mode (this is the best you've got...)

Preface... this is all going to work much much better if you run this on a local network...
    1. launch a hotspot on a laptop natively running ubuntu 22.04 (cell phone's don't seem to work)
        a. wifi settings -> three dots -> turn on wifi hotspot
    2. configure the pi to connect to the hotspot
    3. If you want internet access in this mode, connect a iphone over usb to the computer hosting the hotspot. Ensure the hotspot is enabled

    In theory, this should allow you to run nodes on other devices...



1. Launch the wasd node (ros2 run clanker_hardware wasd.py)
    a. set the tune mode param (ros2 param set wasd_node tune_mode false)
    b. set the pwm mode param (ros2 param set wasd_node pwm_mode false)
2. Launch the rover node (ros2 launch robo_rover rover_launch.py)
    a. wait for the gyro to finish calibrating - check that it is done correctly by (ros2 topic echo /imu/gyro). The anuglar z should be zero if the rover is on the ground
3. Run the slam (ros2 launch estimatation_mapping estimation_launch.py)
4. Now you need to map, drive the robot slowly around the hallway, You need to stop at each "feature" ... hallway ... to let the map update.
    a. make sure you do slightly over 1 lap, you should see loop closure (typically takes approx 1.25 laps)
    b. if you want it to be good run two laps...
5. now you may echo the pose ros2 run tf2_ros tf2 echo map base_link
    a. The pose of the robot is a tf2 frame not a topic
6. kill the wasd node...
    a. now you may run the controller (you need to figure it out from here...)

You need to map because loading the map into slam toolbox doesn't work (this also prohibits pure localization mode)

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
ros2 run tf2_ros tf2_echo map odom\
\
```