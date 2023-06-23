#ifndef _ROS_SERVICE_SetROI_h
#define _ROS_SERVICE_SetROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace camera_control_msgs
{

static const char SETROI[] = "camera_control_msgs/SetROI";

  class SetROIRequest : public ros::Msg
  {
    public:
      typedef sensor_msgs::RegionOfInterest _target_roi_type;
      _target_roi_type target_roi;

    SetROIRequest():
      target_roi()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->target_roi.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->target_roi.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETROI; };
    virtual const char * getMD5() override { return "cf55ea464b4556def55bfcda0d3eab55"; };

  };

  class SetROIResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::RegionOfInterest _reached_roi_type;
      _reached_roi_type reached_roi;
      typedef bool _success_type;
      _success_type success;

    SetROIResponse():
      reached_roi(),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->reached_roi.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->reached_roi.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return SETROI; };
    virtual const char * getMD5() override { return "70a557e11203ac25f1a7d115b99d7d9b"; };

  };

  class SetROI {
    public:
    typedef SetROIRequest Request;
    typedef SetROIResponse Response;
  };

}
#endif
