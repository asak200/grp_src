# grp_src

## Don't forget to `colcon build --symlink-install`

1. Run only the drone inside gazebo:
`ros2 launch system_bringup drone_sim.launch.py`

inside, you'll have a yellow cube. You can change its position by sending the new position with:
`ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"`

2. 

