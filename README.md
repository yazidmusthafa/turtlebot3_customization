Terminal 1:

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 


Terminal 2:

ros2 launch turtlebot3_navigation2 navigation2.launch.py 


Terminal 3:

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: { sec: 0, nanosec: 0 },
    frame_id: 'map'
  },
  pose: {
    position: {x: 1.86875319480896, y: 0.8718680739402771, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: -0.8746434075990963, w: 0.48476686102026506}
  }
}"
