# uav_VFH


## ROS 2 Commands

### Obstacle Avoidance Node
```bash
ros2 run navi obstacle_avoidance

ros2 launch navi camera_bridge.py

ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"