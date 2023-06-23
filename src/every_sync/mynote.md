->启动 
setserial /dev/ttyUSB0 low_latency
roslaunch versavis real_start.launch
->rosserial单独连接触发板 
->rostopic复位触发板 
$ rosrun rosserial_arduino serial_node.py _port:=/dev/versavis _baud:=250000
$ rostopic pub /versavis/reset std_msgs/Bool false --once

Attention:初次上电后需要复位触发板 复位后立即启动real_start.launch
->python 录制数据包
$ rosbag record -O ms_test.bag  /versavis/cam1/image_raw /versavis/cam2/image_raw /versavis/cam1/image_time /versavis/cam2/image_time /versavis/xsens_imu/image_time /versavis/xsens_imu/data

->python 画图评估脚本

-->ros到达时间给定时间戳计算
$ python real_show_timings.py --input_rosbag /home/kang/realvisWs/bag_file/ms_test.bag --topic /versavis/cam2/image_time

-->延迟时间计算：此延迟是积分时间结束与 ROS 驱动程序给出的时间戳之间的时间。 通常，这对应于传感器的最高帧速率（也可能受接口带宽的影响）。
$ python real_calculate_img_delay.py --input_rosbag /home/kang/realvisWs/bag_file/ms_test.bag --cam0 /versavis/cam1/image_raw --cam1 /versavis/cam2/image_raw
->录制轨迹地图数据包
rosbag record -O realvis_track_5_25_1.bag /versavis/xsens_imu/data /versavis/cam1/image_raw /hikrobot_camera/rgb /imu/data /mynteye/left_rect/image_rect /mynteye/imu/data_raw

截取		
		secs:  1653313555
		nsecs:  1081023

		secs:  1653313759
		nsecs:  751216031


rosbag filter realvis_track_5_23_0.bag cut_realvis_track.bag "t.to_sec() >= 1653313555.1081023 and t.to_sec() <= 1653313759.751216031"

rosbag record -O mynt_track.bag /mynteye/left_rect/image_rect /mynteye/imu/data_raw

rosbag record -O orb_track_5_25_1.bag /versavis/xsens_imu/data /versavis/cam1/image_raw /hikrobot_camera/rgb /imu/data /mynteye/left_rect/image_rect /mynteye/imu/data_raw


#轨迹绘制
1. 数据转换
evo_traj euroc data.csv --save_as_tum
2. 轨迹绘制
evo_traj tum vins_result_no_loop.csv -p --plot_mode=xy
evo_traj tum vins_result_loop.csv -p --plot_mode=xy
evo_traj tum 5_25_1mynt/vins_result_no_loop.csv 5_25_1realvis_sync/vins_result_no_loop.csv -p --plot_mode=xy
evo_traj tum 5_25_1mynt/vins_result_no_loop.csv 5_25_1realvis_sync/vins_result_no_loop.csv -p --plot_mode=xy

xsens mti ros driver attention!!!
The published topics do not reach the expected data output rate (e.g. 400 Hz).
We have noticed that the ROS node can cause a high CPU load, leading to lower data output rates. This issue has been fixed in ROS nodes available in MTSS2019.3.2 and later. We recommend migrating to the latest version.

If you have connected the MTi via a USB interface, we recommend enabling the low latency mode using setserial:

Install setserial if not already installed
Enable low latency mode:

$ setserial [/path/to/xsens/port] low_latency
setserial /dev/ttyUSB0 low_latency

Check the output rate of the published topics, e.g:

$ rostopic hz imu/data

imu timestamp

		nsecs:  462945000
		nsecs:  472942000
		nsecs:  482940000
		nsecs:  492938000
均匀
bag1
image timestamp 1
		nsecs:  101840000
		nsecs:  149825000
		nsecs:  197811000

image timestamp 2
		nsecs:  149967000
		nsecs:  197953000
		nsecs:  245938000

