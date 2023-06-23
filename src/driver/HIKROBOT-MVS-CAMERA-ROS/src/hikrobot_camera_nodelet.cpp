#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"
#include <image_numbered_msgs/ImageNumbered.h>

//* 必要头文件
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h> 

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

uint64 seq_num=0;

namespace hikRobotCamera_nodelet_ns
{
class HikRobotCameraNodelet : public nodelet::Nodelet //* 任何nodelet plugin都要继承Nodelet类
{
public:
    HikRobotCameraNodelet() = default;

private:
    virtual  void onInit() override
    {
        //********** variables    **********/
        cv::Mat src;
        //string src = "",image_pub = "";
        
        ros::NodeHandle hikrobot_camera;
        camera::Camera MVS_cap(hikrobot_camera);

        image_transport::ImageTransport main_cam_image(hikrobot_camera);
        image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);
        new_pub_ = hikrobot_camera.advertise<image_numbered_msgs::ImageNumbered>("/hikrobot_camera/img_numbered", 1000);//added 2022.5.6
        
        cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
        cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 

        //********** 10 Hz        **********/
        ros::Rate loop_rate(20);//?????原始为10hz 需要更改匹配frame rate
        //外部触发设定频率多少，此参数亦为多少，暂时需要手动修改

        while (ros::ok())
        {

            loop_rate.sleep();
            ros::spinOnce();

            MVS_cap.ReadImg(src);
            if (src.empty())
            {
                continue;
            }
    #if FIT_LIDAR_CUT_IMAGE
            cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
            cv::Mat src_new = src(area);
            cv_ptr->image = src_new;
    #else
            cv_ptr->image = src;
    #endif
            image_msg = *(cv_ptr->toImageMsg());
            image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
            image_msg.header.frame_id = "hikrobot_camera";

            camera_info_msg.header.frame_id = image_msg.header.frame_id;
            camera_info_msg.header.stamp = image_msg.header.stamp;
            image_pub.publish(image_msg, camera_info_msg);

            
            image_numbered_msgs::ImageNumbered IN_msg;
            
            IN_msg.image = image_msg;
            IN_msg.number = seq_num;
            IN_msg.exposure = 0;
            new_pub_.publish(IN_msg);
            
            seq_num ++;      

            //*******************************************************************************************************************/
        }
    }

    image_transport::CameraPublisher image_pub;
    ros::Publisher new_pub_;
    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
}
PLUGINLIB_EXPORT_CLASS(hikRobotCamera_nodelet_ns::HikRobotCameraNodelet, nodelet::Nodelet)
}
