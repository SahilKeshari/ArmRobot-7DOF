
## To run the robot with Gazebo.
```bash
ros2 launch new_cobot_moveit gazebo.launch.py
```

## To launch the Rviz and controllers.
```bash
ros2 launch new_cobot_moveit demo.launch.py
```

## To run the node which publishes the end effector velocity under topic `/eef_velocity`.
```bash
ros2 run new_cobot_moveit ee_velocity
```

## To run the node which publishes cartesian movements to the robot.
```bash
ros2 run new_cobot_moveit multi_waypoint_cartesian
```
