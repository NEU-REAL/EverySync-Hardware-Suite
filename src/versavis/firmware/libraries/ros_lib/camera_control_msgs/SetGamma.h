#ifndef _ROS_SERVICE_SetGamma_h
#define _ROS_SERVICE_SetGamma_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETGAMMA[] = "camera_control_msgs/SetGamma";

  class SetGammaRequest : public ros::Msg
  {
    public:
      typedef float _target_gamma_type;
      _target_gamma_type target_gamma;

    SetGammaRequest():
      target_gamma(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_gamma;
      u_target_gamma.real = this->target_gamma;
      *(outbuffer + offset + 0) = (u_target_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_gamma);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_gamma;
      u_target_gamma.base = 0;
      u_target_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_gamma = u_target_gamma.real;
      offset += sizeof(this->target_gamma);
     return offset;
    }

    virtual const char * getType() override { return SETGAMMA; };
    virtual const char * getMD5() override { return "0a9682c360c7e6178269a064cdf9a0c9"; };

  };

  class SetGammaResponse : public ros::Msg
  {
    public:
      typedef float _reached_gamma_type;
      _reached_gamma_type reached_gamma;
      typedef bool _success_type;
      _success_type success;

    SetGammaResponse():
      reached_gamma(0),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_reached_gamma;
      u_reached_gamma.real = this->reached_gamma;
      *(outbuffer + offset + 0) = (u_reached_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_gamma);
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
      } u_reached_gamma;
      u_reached_gamma.base = 0;
      u_reached_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_gamma = u_reached_gamma.real;
      offset += sizeof(this->reached_gamma);
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

    virtual const char * getType() override { return SETGAMMA; };
    virtual const char * getMD5() override { return "6f5a47cc339c639a71a650af05aed3b9"; };

  };

  class SetGamma {
    public:
    typedef SetGammaRequest Request;
    typedef SetGammaResponse Response;
  };

}
#endif
