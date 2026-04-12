# grp_src

## Don't forget to `colcon build --symlink-install`

1. Run only the drone inside gazebo:

`ros2 launch system_bringup drone_sim.launch.py`

inside, you'll have a yellow cube. You can change its position by sending the new position with:

`ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}}"`

2. Only run the SSMR:

`ros2 launch system_bringup ssmr_sim.launch.py`

To control the SSMR's velocity: 
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped`


3. Run the SSMR and the Drone:

`ros2 launch system_bringup ssmr_sim.launch.py drone:=true`

### To run the segmentation & path planning model:

`ros2 run perception_pkg vision_Nav`