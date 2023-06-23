#include <ros/ros.h>
#include <boost/thread.hpp>
#include <pylon_camera/pylon_camera_node.h>
#include <image_numbered_msgs/ImageNumbered.h>

//* 必要头文件
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

// uint64 seq_num=0;
// ros::Publisher ImgNumbered_pub_;
// image_numbered_msgs::ImageNumbered IN_msg;

namespace pylon_camera_ns
{
    class Nodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit()
            {

                boost::shared_ptr<boost::thread> spinThread_;
                
                // pylon_camera::PylonCameraNode pylon_camera_node;
                pylon_camera::PylonCameraNode pylon_camera_node(getPrivateNodeHandle());
                
                // ros::NodeHandle basler_camera;
                // ImgNumbered_pub_ = basler_camera.advertise<image_numbered_msgs::ImageNumbered>("/basler_camera_node/img_numbered", 1000);

                ros::Rate r(pylon_camera_node.frameRate());
                
                ROS_INFO_STREAM("Start image grabbing if node connects to topic with "
                    << "a frame_rate of: " << pylon_camera_node.frameRate() << " Hz");

                // spinThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&pylon_camera_ns::spin, this)));
                // Main thread and brightness-service thread
                
                // boost::thread th(boost::bind(&ros::spin));

                while ( ros::ok() )
                {
                    pylon_camera_node.spin();
                    
                    // IN_msg.image = pylon_camera_node.img_raw_msg_;
                    // IN_msg.number = seq_num;
                    // IN_msg.exposure = 0;S
                    // new_pub_.publish(IN_msg);
                    // seq_num ++;
                    r.sleep();
                }

                ROS_INFO("Terminate PylonCameraNode");
                // return EXIT_SUCCESS;
            }



            // void PylonCameraNodelet::spin()
            // {
            //     while (true)
            //     {
            //         ros::Rate r(pylon_camera_node.frameRate());
            //         while (ros::ok() )
            //         {
            //             pylon_camera_node.spin();
            //             IN_msg.image = pylon_camera_node.img_raw_msg_;
            //             IN_msg.number = seq_num;
            //             IN_msg.exposure = 0;
            //             new_pub_.publish(IN_msg);
            //             // seq_num ++;

            //             r.sleep();
            //         }
            //     }
            // }
    };
    PLUGINLIB_EXPORT_CLASS(pylon_camera_ns::Nodelet, nodelet::Nodelet)
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "pylon_camera_node");
//     pylon_camera::PylonCameraNode pylon_camera_node;
    
//     //新增可发布image_numbered话题 
//     ros::init(argc, argv, "basler_camera");
//     ros::NodeHandle basler_camera;
//     ros::Publisher new_pub_ = basler_camera.advertise<image_numbered_msgs::ImageNumbered>("/basler_camera_node/img_numbered", 1000);
    

//     ros::Rate r(pylon_camera_node.frameRate());

//     ROS_INFO_STREAM("Start image grabbing if node connects to topic with "
//         << "a frame_rate of: " << pylon_camera_node.frameRate() << " Hz");

//     // Main thread and brightness-service thread
//     boost::thread th(boost::bind(&ros::spin));

//     while ( ros::ok() )
//     {
//         pylon_camera_node.spin();
        
//         image_numbered_msgs::ImageNumbered IN_msg;
        
//         IN_msg.image = pylon_camera_node.img_raw_msg_;
//         IN_msg.number = seq_num;
//         IN_msg.exposure = 0;
//         new_pub_.publish(IN_msg);
//         // seq_num ++;

//         r.sleep();
//     }

//     ROS_INFO("Terminate PylonCameraNode");
//     return EXIT_SUCCESS;
// }