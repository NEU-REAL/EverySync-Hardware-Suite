#ifndef ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_
#define ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_

#include <cmath>
#include <mutex>
#include <sstream>
#include <string>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>

#include <image_numbered_msgs/ImageNumbered.h>
#include <imu_numbered_msgs/ImuNumbered.h>

#include <every_sync/ImuMicro.h>
#include <every_sync/TimeNumbered.h>
#include <ext_clk_msgs/LocalSensorExternalTrigger.h>

namespace every_sync {
class Synchronizer {
public:
  Synchronizer(const ros::NodeHandle &nh,
                       const ros::NodeHandle &nh_private);
  ~Synchronizer();

  void imageCallback(const image_numbered_msgs::ImageNumbered &image_msg);
  // void imuTimeCallback(const imu_numbered_msgs::ImuNumbered &imu_msg);
  void imuCallback(const imu_numbered_msgs::ImuNumbered &imu_msg);//added 2022.5.6 by kang
  void ext_clkCallback(const every_sync::TimeNumbered &ext_clk_time_msg);//added 2022.5.6 by kang
  void cameraInfoCallback(const sensor_msgs::CameraInfo& camera_info_msg);
  void imageTimeCallback(const every_sync::TimeNumbered &image_time_msg);
  void imuTimeCallback(const every_sync::TimeNumbered &imu_time_msg);//added 2022.5.6 by kang
  void publishImg(const image_numbered_msgs::ImageNumbered &image_msg);
  void publishImu(const imu_numbered_msgs::ImuNumbered &imu_msg);//added 2022.5.6 by kang

  bool readParameters();

  // Find nearest time stamp in a list of candidates (only newer are possible
  // due to driver logic).
  void associateTimeStampsAndCleanUp();
  void Imu_associateTimeStampsAndCleanUp();

private:
  // ROS members.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  image_transport::ImageTransport image_transport_;
  ros::Subscriber image_sub_;
  ros::Subscriber xsens_imu_sub_;//added 2022.5.6 by kang
  ros::Subscriber ext_clk_sub_;//added 2023.5.10 by kang
  ros::Publisher ext_clk_pub_;//added 2023.5.10 by kang
  ros::Subscriber camera_info_sub_;
  image_transport::Publisher image_fast_pub_;
  image_transport::Publisher image_slow_pub_;
  ros::Publisher imu_fast_pub_;//added 2022.5.6 by kang
  ros::Publisher camera_info_fast_pub_;
  ros::Publisher camera_info_slow_pub_;
  ros::Publisher initialized_pub_;
  ros::Subscriber image_time_sub_;
  ros::Subscriber imu_time_sub_;

  // Topic names.
  std::string every_sync_topic_;
  std::string driver_topic_;
  std::string imu_driver_topic_;//added 2022.5.6 by kang
  std::string camera_info_sub_topic_;
  std::string image_fast_pub_topic_;
  std::string image_slow_pub_topic_;
  std::string camera_info_fast_pub_topic_;
  std::string camera_info_slow_pub_topic_;
  std::string image_time_sub_topic_;
  std::string imu_time_sub_topic_;//added 2022.5.6 by kang
  std::string initialized_pub_topic_;
  std::string imu_fast_pub_topic_;//added 2022.5.6 by kang
  //enable ext clk input
  std::string ext_clk_sub_topic_;//added 2023.5.10 by kang
  std::string ext_clk_pub_topic_;//added 2022.5.10 by kang

  // Association members.
  every_sync::TimeNumbered init_time_;
  std::vector<every_sync::TimeNumbered> image_time_stamp_candidates_;
  std::vector<image_numbered_msgs::ImageNumbered> image_candidates_;
  std::vector<every_sync::TimeNumbered> imu_time_stamp_candidates_;//added 2022.5.6 by kang
  std::vector<imu_numbered_msgs::ImuNumbered> imu_candidates_;//added 2022.5.6 by kang
  bool received_first_camera_info_;
  sensor_msgs::CameraInfo camera_info_msg_;
  ros::Time last_stamp_;
  ros::Time init_timestamp_;

  // Constants.
  const uint64_t kMaxImageCandidateLength;
  const uint8_t kMinSuccessfullConsecutiveMatches;
  const uint8_t kMinSuccessfullConsecutiveMatches_imu;
  const double kMaxImageDelayThreshold;
  const double kMaxImageDelayThreshold_imu; 
  // Image numbers and initialization.
  uint8_t init_number_;
  uint64_t last_image_number_ = 0;
  uint64_t last_imu_number_ = 0;
  int64_t offset_;
  bool initialized_;

  // Configuration.
  uint8_t slow_publisher_image_counter_;
  bool publish_slow_images_;
  int publish_every_n_image_;
  bool forward_camera_info_;
  ros::Duration imu_offset_;

  uint64_t ext_clk_count;

  int sum_num = 0;
  double sum_time_trans = 0.0;
  int imu_sum_num = 0;
  double imu_sum_time_trans = 0.0;

  int exposure_time = 0;
  std::mutex mutex_;
};
} // namespace every_sync

#endif // ARDUINO_VI_SYNC_ARDUINO_VI_SYNCHRONIZER_H_
