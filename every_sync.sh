setserial /dev/ttyUSB0 low_latency;
# setserial /dev/ttyUSB1 low_latency;

source devel/setup.bash;

# roslaunch realvis real_start_basler_xsens.launch;
roslaunch every_sync real_start_basler_xsens.launch;
