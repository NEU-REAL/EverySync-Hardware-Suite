#ifndef _ROS_SERVICE_SetGain_h
#define _ROS_SERVICE_SetGain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETGAIN[] = "camera_control_msgs/SetGain";

  class SetGainRequest : public ros::Msg
  {
    public:
      typedef float _target_gain_type;
      _target_gain_type target_gain;

    SetGainRequest():
      target_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_gain;
      u_target_gain.real = this->target_gain;
      *(outbuffer + offset + 0) = (u_target_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target_gain;
      u_target_gain.base = 0;
      u_target_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_gain = u_target_gain.real;
      offset += sizeof(this->target_gain);
     return offset;
    }

    virtual const char * getType() override { return SETGAIN; };
    virtual const char * getMD5() override { return "5e755c5a3e141e0e1df554ec3425596f"; };

  };

  class SetGainResponse : public ros::Msg
  {
    public:
      typedef float _reached_gain_type;
      _reached_gain_type reached_gain;
      typedef bool _success_type;
      _success_type success;

    SetGainResponse():
      reached_gain(0),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_reached_gain;
      u_reached_gain.real = this->reached_gain;
      *(outbuffer + offset + 0) = (u_reached_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_gain);
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
      } u_reached_gain;
      u_reached_gain.base = 0;
      u_reached_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_gain = u_reached_gain.real;
      offset += sizeof(this->reached_gain);
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

    virtual const char * getType() override { return SETGAIN; };
    virtual const char * getMD5() override { return "885668c9ebfd53815071a529456c4b42"; };

  };

  class SetGain {
    public:
    typedef SetGainRequest Request;
    typedef SetGainResponse Response;
  };

}
#endif
