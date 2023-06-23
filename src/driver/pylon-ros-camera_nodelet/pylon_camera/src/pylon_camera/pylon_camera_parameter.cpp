/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Improved by drag and bot GmbH (www.dragandbot.com), 2019
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <pylon_camera/pylon_camera_parameter.h>
#include <sensor_msgs/image_encodings.h>

namespace pylon_camera
{

PylonCameraParameter::PylonCameraParameter() :
        camera_frame_("pylon_camera"),
        device_user_id_(""),
        frame_rate_(20.0),
        camera_info_url_(""),
        image_encoding_("rgb8"),
        binning_x_(1),
        binning_y_(1),
        binning_x_given_(false),
        binning_y_given_(false),
        downsampling_factor_exp_search_(1),
        // ##########################
        //  image intensity settings
        // ##########################
        exposure_(25000.0),
        exposure_given_(false),
        gain_(0.5),
        gain_given_(false),
        gamma_(1.0),
        gamma_given_(false),
        brightness_(100),
        brightness_given_(false),
        brightness_continuous_(false),
        exposure_auto_(true),
        gain_auto_(true),
        // #########################
        exposure_search_timeout_(5.),
        auto_exp_upper_lim_(0.0),
        mtu_size_(3000),
        enable_status_publisher_(false),
        enable_current_params_publisher_(false),
        inter_pkg_delay_(1000),
        startup_user_set_("UserSet1"),
        shutter_mode_(SM_DEFAULT),
        auto_flash_(false), 
        grab_timeout_(10000),
        trigger_timeout_(20000),
	    grab_strategy_(1),
        white_balance_auto_(1),
        white_balance_auto_given_(false),
        white_balance_ratio_red_(1.0),
        white_balance_ratio_green_(1.0),
        white_balance_ratio_blue_(1.0),
        white_balance_ratio_given_(false)
{}

PylonCameraParameter::~PylonCameraParameter()
{}

void PylonCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
{
    nh.param<std::string>("camera_frame", camera_frame_, "pylon_camera");
    std::cout << "############################################################################" << std::endl;
    std::cout << "The camera_frame is " << camera_frame_ << std::endl;
    std::cout << "############################################################################" << std::endl;

    nh.param<std::string>("device_user_id", device_user_id_, "");

    if ( nh.hasParam("frame_rate") )
    {
        nh.getParam("frame_rate", frame_rate_);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The frame_rate is " << frame_rate_ << std::endl;
        std::cout << "############################################################################" << std::endl;
    }

    nh.param<std::string>("camera_info_url", camera_info_url_, "");
    if ( nh.hasParam("camera_info_url") )
    {
        nh.getParam("camera_info_url", camera_info_url_);
    }

    binning_x_given_ = nh.hasParam("binning_x");
    if ( binning_x_given_ )
    {
        int binning_x;
        nh.getParam("binning_x", binning_x);
        ROS_DEBUG_STREAM("binning x is given and has value " << binning_x);
        if ( binning_x > 32 || binning_x < 0 )
        {
            ROS_WARN_STREAM("Desired horizontal binning_x factor not in valid "
                << "range! Binning x = " << binning_x << ". Will reset it to "
                << "default value (1)");
            binning_x_given_ = false;
        }
        else
        {
            binning_x_ = static_cast<size_t>(binning_x);
        }
    }
    binning_y_given_ = nh.hasParam("binning_y");
    if ( binning_y_given_ )
    {
        int binning_y;
        nh.getParam("binning_y", binning_y);
        ROS_DEBUG_STREAM("binning y is given and has value " << binning_y);
        if ( binning_y > 32 || binning_y < 0 )
        {
            ROS_WARN_STREAM("Desired vertical binning_y factor not in valid "
                << "range! Binning y = " << binning_y << ". Will reset it to "
                << "default value (1)");
            binning_y_given_ = false;
        }
        else
        {
            binning_y_ = static_cast<size_t>(binning_y);
        }
    }
    nh.param<int>("downsampling_factor_exposure_search",
                  downsampling_factor_exp_search_,
                  20);

    if ( nh.hasParam("image_encoding") )
    {
        std::string encoding;
        nh.getParam("image_encoding", encoding);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The image_encoding is " << encoding << std::endl;
        std::cout << "############################################################################" << std::endl;
        if ( !encoding.empty() &&
             !sensor_msgs::image_encodings::isMono(encoding) &&
             !sensor_msgs::image_encodings::isColor(encoding) &&
             !sensor_msgs::image_encodings::isBayer(encoding) &&
             encoding != sensor_msgs::image_encodings::YUV422 )
        {
            ROS_WARN_STREAM("Desired image encoding parameter: '" << encoding
                << "' is not part of the 'sensor_msgs/image_encodings.h' list!"
                << " Will not set encoding");
            encoding = std::string("");
        }
        image_encoding_ = encoding;
    }

    // ##########################
    //  image intensity settings
    // ##########################

    // > 0: Exposure time in microseconds
    exposure_given_ = nh.hasParam("exposure");
    if ( exposure_given_ )
    {
        nh.getParam("exposure", exposure_);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The exposure is " << exposure_ << std::endl;
        std::cout << "############################################################################" << std::endl;
        ROS_DEBUG_STREAM("exposure is given and has value " << exposure_);
    }

    gain_given_ = nh.hasParam("gain");
    if ( gain_given_ )
    {
        nh.getParam("gain", gain_);
        ROS_DEBUG_STREAM("gain is given and has value " << gain_);
    }

    gamma_given_ = nh.hasParam("gamma");
    if ( gamma_given_ )
    {
        nh.getParam("gamma", gamma_);
        ROS_DEBUG_STREAM("gamma is given and has value " << gamma_);
    }

    brightness_given_ = nh.hasParam("brightness");
    if ( brightness_given_ )
    {
        nh.getParam("brightness", brightness_);
        ROS_DEBUG_STREAM("brightness is given and has value " << brightness_);
        if ( gain_given_ && exposure_given_ )
        {
            ROS_WARN_STREAM("Gain ('gain') and Exposure Time ('exposure') "
                << "are given as startup ros-parameter and hence assumed to be "
                << "fix! The desired brightness (" << brightness_ << ") can't "
                << "be reached! Will ignore the brightness by only "
                << "setting gain and exposure . . .");
            brightness_given_ = false;
        }
        else
        {
            if ( nh.hasParam("brightness_continuous") )
            {
                nh.getParam("brightness_continuous", brightness_continuous_);
                ROS_DEBUG_STREAM("brightness is continuous");
            }
            if ( nh.hasParam("exposure_auto") )
            {
                nh.getParam("exposure_auto", exposure_auto_);
                ROS_DEBUG_STREAM("exposure is set to auto");
            }
            if ( nh.hasParam("gain_auto") )
            {
                nh.getParam("gain_auto", gain_auto_);
                ROS_DEBUG_STREAM("gain is set to auto");
            }
        }
    }
    // ##########################

    nh.param<double>("exposure_search_timeout", exposure_search_timeout_, 5.);
    nh.param<double>("auto_exposure_upper_limit", auto_exp_upper_lim_, 10000000.);

    if ( nh.hasParam("gige/mtu_size") )
    {
        nh.getParam("gige/mtu_size", mtu_size_);
    }

    if ( nh.hasParam("enable_status_publisher") )
    {
        nh.getParam("enable_status_publisher", enable_status_publisher_);
    }

    if ( nh.hasParam("enable_current_params_publisher") )
    {
        nh.getParam("enable_current_params_publisher", enable_current_params_publisher_);
    }

    if ( nh.hasParam("gige/inter_pkg_delay") )
    {
        nh.getParam("gige/inter_pkg_delay", inter_pkg_delay_);
    }

    std::string shutter_param_string;
    nh.param<std::string>("shutter_mode", shutter_param_string, "");
    if ( shutter_param_string == "rolling" )
    {
        shutter_mode_ = SM_ROLLING;
    }
    else if ( shutter_param_string == "global" )
    {
        shutter_mode_ = SM_GLOBAL;
    }
    else if ( shutter_param_string == "global_reset" )
    {
        shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
        shutter_mode_ = SM_DEFAULT;
    }
    if ( nh.hasParam("startup_user_set") )
    {
        nh.getParam("startup_user_set", startup_user_set_);
    }
    if ( nh.hasParam("grab_timeout") )
    {
        nh.getParam("grab_timeout", grab_timeout_);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The grab_timeout is " << grab_timeout_ << std::endl;
        std::cout << "############################################################################" << std::endl;
    }
    if ( nh.hasParam("trigger_timeout") )
    {
        nh.getParam("trigger_timeout", trigger_timeout_);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The trigger_timeout is " << trigger_timeout_ << std::endl;
        std::cout << "############################################################################" << std::endl;
    }
    if ( nh.hasParam("white_balance_auto") )
    {
        nh.getParam("white_balance_auto", white_balance_auto_);
        white_balance_auto_given_ = true;
    }
    if ( nh.hasParam("white_balance_ratio_red") )
    {
        nh.getParam("white_balance_ratio_red", white_balance_ratio_red_);
        white_balance_ratio_given_ = true;
    }
    if ( nh.hasParam("white_balance_ratio_green") )
    {
        nh.getParam("white_balance_ratio_green", white_balance_ratio_green_);
        white_balance_ratio_given_ = true;
    }
    if ( nh.hasParam("white_balance_ratio_blue") )
    {
        nh.getParam("white_balance_ratio_blue", white_balance_ratio_blue_);
        white_balance_ratio_given_ = true;
    }
    if ( nh.hasParam("grab_strategy") )
    {
        nh.getParam("grab_strategy", grab_strategy_);
        std::cout << "############################################################################" << std::endl;
        std::cout << "The grab_strategy is " << grab_strategy_ << std::endl;
        std::cout << "############################################################################" << std::endl;
    }

    nh.param<bool>("auto_flash", auto_flash_, false);
    nh.param<bool>("auto_flash_line_2", auto_flash_line_2_, true);
    nh.param<bool>("auto_flash_line_3", auto_flash_line_3_, true);

    ROS_WARN("Autoflash: %i, line2: %i , line3: %i ", auto_flash_, auto_flash_line_2_, auto_flash_line_3_);
    validateParameterSet(nh);
    return;
}

void PylonCameraParameter::adaptDeviceUserId(const ros::NodeHandle& nh, const std::string& device_user_id)
{
    device_user_id_ = device_user_id;
    nh.setParam("device_user_id", device_user_id_);
}

void PylonCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
{
    if ( !device_user_id_.empty() )
    {
        ROS_INFO_STREAM("Trying to open the following camera: "
            << device_user_id_.c_str());
    }
    else
    {
        ROS_INFO_STREAM("No Device User ID set -> Will open the camera device "
                << "found first");
    }

    if ( frame_rate_ < 0 && frame_rate_ != -1 )
    {
        ROS_WARN_STREAM("Unexpected frame rate (" << frame_rate_ << "). Will "
                << "reset it to default value which is 5 Hz");
        frame_rate_ = 5.0;
        nh.setParam("frame_rate", frame_rate_);
    }

    if ( exposure_given_ && ( exposure_ <= 0.0 || exposure_ > 1e7 ) )
    {
        ROS_WARN_STREAM("Desired exposure measured in microseconds not in "
                << "valid range! Exposure time = " << exposure_ << ". Will "
                << "reset it to default value!");
        exposure_given_ = false;
    }

    if ( gain_given_ && ( gain_ < 0.0 || gain_ > 1.0 ) )
    {
        ROS_WARN_STREAM("Desired gain (in percent) not in allowed range! "
                << "Gain = " << gain_ << ". Will reset it to default value!");
        gain_given_ = false;
    }

    if ( brightness_given_ && ( brightness_ < 0.0 || brightness_ > 255 ) )
    {
        ROS_WARN_STREAM("Desired brightness not in allowed range [0 - 255]! "
               << "Brightness = " << brightness_ << ". Will reset it to "
               << "default value!");
        brightness_given_ = false;
    }

    if ( exposure_search_timeout_ < 5.)
    {
        ROS_WARN_STREAM("Low timeout for exposure search detected! Exposure "
            << "search may fail.");
    }
    return;
}

const std::string& PylonCameraParameter::deviceUserID() const
{
    return device_user_id_;
}

std::string PylonCameraParameter::shutterModeString() const
{
    if ( shutter_mode_ == SM_ROLLING )
    {
        return "rolling";
    }
    else if ( shutter_mode_ == SM_GLOBAL )
    {
        return "global";
    }
    else if ( shutter_mode_ == SM_GLOBAL_RESET_RELEASE )
    {
        return "global_reset";
    }
    else
    {
        return "default_shutter_mode";
    }
}

const std::string& PylonCameraParameter::imageEncoding() const
{
    return image_encoding_;
}

bool PylonCameraParameter::setimageEncodingParam(const ros::NodeHandle& nh, const std::string& format) 
{
    try
    {
        image_encoding_ = format;
        nh.setParam("image_encoding", image_encoding_);
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error Saving the new image encoding as ROS Param");
        return false;
    }
}

const std::string& PylonCameraParameter::cameraFrame() const
{
    return camera_frame_;
}

const double& PylonCameraParameter::frameRate() const
{
    return frame_rate_;
}

void PylonCameraParameter::setFrameRate(const ros::NodeHandle& nh,
                                        const double& frame_rate)
{
    frame_rate_ = frame_rate;
    nh.setParam("frame_rate", frame_rate_);
}

const std::string& PylonCameraParameter::cameraInfoURL() const
{
    return camera_info_url_;
}

void PylonCameraParameter::setCameraInfoURL(const ros::NodeHandle& nh,
                                            const std::string& camera_info_url)
{
    camera_info_url_ = camera_info_url;
    nh.setParam("camera_info_url", camera_info_url_);
}

}  // namespace pylon_camera
