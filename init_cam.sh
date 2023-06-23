source devel/setup.bash
rostopic pub -1 /realvis/cam2/init std_msgs/Bool "data: true"
rostopic pub -1 /realvis/cam1/init std_msgs/Bool "data: true"
rostopic pub -1 /realvis/imu/init std_msgs/Bool "data: true"
rostopic pub -1 /realvis/lidar/init std_msgs/Bool "data: true"
