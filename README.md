# Object following and tracking using UR5 robot arm

## Setup
```
cd ~/cobot_ws
catkin build
```

## Run simulation
```
roslaunch ur5_moveit_package sim.launch
cd ~/cobot_ws/src/scripts
python3 ar_detect.py  % use ar_detect_with_kf.py for kalman filtering
python3 init_ur5_pose.py 
python3 track_marker.py
```

### To move the wheely robot
```
python3 wheel_robot_controller.py
```
