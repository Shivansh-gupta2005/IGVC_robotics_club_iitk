# Imp commands
After cloning the repo

1) Go to the workspace  
```cd IGVC_robotics_club_iitk ```

3) Delete the build and log files  
```rm -rf build log```

4) Export command to properly load the world  
```export GAZEBO_MODEL_PATH=/<path>/<to_your>/<workspace_from_home_directory>/src/two_wheel_drive:$GAZEBO_MODEL_PATH```

5) build the workspace  
```colcon build --symlink-install```

6) source it  
```source install/setup.bash```

7) launch  
```ros2 launch two_wheel_drive robot.launch.py```

8) For viewing the RGB-D camera's live feed run this node
```python3 src/depth_camera_subscriber.py```

9) start ekf filter or ukf filter on imu and odom and gps

```ros2 launch robot_localization ekf.launch.py```

```ros2 launch robot_localization navsat_transform.launch.py```

