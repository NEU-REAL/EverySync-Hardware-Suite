#ifndef _ROS_SERVICE_SetExposure_h
#define _ROS_SERVICE_SetExposure_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETEXPOSURE[] = "camera_control_msgs/SetExposure";

  class SetExposureRequest : public ros::Msg
  {
    public:
      typedef float _target_exposure_type;
      _target_exposure_type target_exposure;

    SetExposureRequest():
      target_exposure(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_exposure;
      u_target_exposure.real = this->target_exposure;
      *(outbuffer + offset + 0) = (u_target_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_exposure);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_exposure;
      u_target_exposure.base = 0;
      u_target_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_exposure = u_target_exposure.real;
      offset += sizeof(this->target_exposure);
     return offset;
    }

    virtual const char * getType() override { return SETEXPOSURE; };
    virtual const char * getMD5() override { return "881807b9f62e6919695085297c872d67"; };

  };

  class SetExposureResponse : public ros::Msg
  {
    public:
      typedef float _reached_exposure_type;
      _reached_exposure_type reached_exposure;
      typedef bool _success_type;
      _success_type success;

    SetExposureResponse():
      reached_exposure(0),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_reached_exposure;
      u_reached_exposure.real = this->reached_exposure;
      *(outbuffer + offset + 0) = (u_reached_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_exposure);
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
      union {
        float real;
        uint32_t base;
      } u_reached_exposure;
      u_reached_exposure.base = 0;
      u_reached_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_exposure = u_reached_exposure.real;
      offset += sizeof(this->reached_exposure);
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

    virtual const char * getType() override { return SETEXPOSURE; };
    virtual const char * getMD5() override { return "f0624b49b89ef97572f8db48c91665b2"; };

  };

  class SetExposure {
    public:
    typedef SetExposureRequest Request;
    typedef SetExposureResponse Response;
  };

}
#endif
