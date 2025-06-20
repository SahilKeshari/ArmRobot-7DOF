
### To run the robot with Gazebo.
```bash
ros2 launch new_cobot_moveit gazebo.launch.py
```

### To launch the Rviz and controllers.
```bash
ros2 launch new_cobot_moveit demo.launch.py
```

### To run the node which publishes the end effector velocity under topic `/eef_velocity`.
```bash
ros2 run new_cobot_moveit ee_velocity
```

### To run the node which publishes cartesian movements to the robot.
```bash
ros2 run new_cobot_moveit multi_waypoint_cartesian
```


### Note: Before running put these inside the `.bashrc` file, helps the Gazebo to find the meshes. Replace `cobot_ws` with the name of your workspace.
#### To Open .bashrc
```
gedit ~/.bashrc
```

* Copy these lines at the end of the file.

```
source ~/cobot_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix new_cobot_moveit)/share
```
