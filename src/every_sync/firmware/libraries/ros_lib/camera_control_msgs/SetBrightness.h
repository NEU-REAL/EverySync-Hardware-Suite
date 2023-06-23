#ifndef _ROS_SERVICE_SetBrightness_h
#define _ROS_SERVICE_SetBrightness_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char SETBRIGHTNESS[] = "camera_control_msgs/SetBrightness";

  class SetBrightnessRequest : public ros::Msg
  {
    public:
      typedef int32_t _target_brightness_type;
      _target_brightness_type target_brightness;
      typedef bool _brightness_continuous_type;
      _brightness_continuous_type brightness_continuous;
      typedef bool _exposure_auto_type;
      _exposure_auto_type exposure_auto;
      typedef bool _gain_auto_type;
      _gain_auto_type gain_auto;

    SetBrightnessRequest():
      target_brightness(0),
      brightness_continuous(0),
      exposure_auto(0),
      gain_auto(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_target_brightness;
      u_target_brightness.real = this->target_brightness;
      *(outbuffer + offset + 0) = (u_target_brightness.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_brightness.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_brightness.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_brightness.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_brightness);
      union {
        bool real;
        uint8_t base;
      } u_brightness_continuous;
      u_brightness_continuous.real = this->brightness_continuous;
      *(outbuffer + offset + 0) = (u_brightness_continuous.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->brightness_continuous);
      union {
        bool real;
        uint8_t base;
      } u_exposure_auto;
      u_exposure_auto.real = this->exposure_auto;
      *(outbuffer + offset + 0) = (u_exposure_auto.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->exposure_auto);
      union {
        bool real;
        uint8_t base;
      } u_gain_auto;
      u_gain_auto.real = this->gain_auto;
      *(outbuffer + offset + 0) = (u_gain_auto.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gain_auto);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_target_brightness;
      u_target_brightness.base = 0;
      u_target_brightness.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_brightness.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_brightness.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_brightness.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_brightness = u_target_brightness.real;
      offset += sizeof(this->target_brightness);
      union {
        bool real;
        uint8_t base;
      } u_brightness_continuous;
      u_brightness_continuous.base = 0;
      u_brightness_continuous.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->brightness_continuous = u_brightness_continuous.real;
      offset += sizeof(this->brightness_continuous);
      union {
        bool real;
        uint8_t base;
      } u_exposure_auto;
      u_exposure_auto.base = 0;
      u_exposure_auto.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->exposure_auto = u_exposure_auto.real;
      offset += sizeof(this->exposure_auto);
      union {
        bool real;
        uint8_t base;
      } u_gain_auto;
      u_gain_auto.base = 0;
      u_gain_auto.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gain_auto = u_gain_auto.real;
      offset += sizeof(this->gain_auto);
     return offset;
    }

    virtual const char * getType() override { return SETBRIGHTNESS; };
    virtual const char * getMD5() override { return "1f8cc426d8d4e41959f6306162a714c3"; };

  };

  class SetBrightnessResponse : public ros::Msg
  {
    public:
      typedef int32_t _reached_brightness_type;
      _reached_brightness_type reached_brightness;
      typedef float _reached_exposure_time_type;
      _reached_exposure_time_type reached_exposure_time;
      typedef float _reached_gain_value_type;
      _reached_gain_value_type reached_gain_value;
      typedef bool _success_type;
      _success_type success;

    SetBrightnessResponse():
      reached_brightness(0),
      reached_exposure_time(0),
      reached_gain_value(0),
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_reached_brightness;
      u_reached_brightness.real = this->reached_brightness;
      *(outbuffer + offset + 0) = (u_reached_brightness.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_brightness.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_brightness.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_brightness.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_brightness);
      union {
        float real;
        uint32_t base;
      } u_reached_exposure_time;
      u_reached_exposure_time.real = this->reached_exposure_time;
      *(outbuffer + offset + 0) = (u_reached_exposure_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_exposure_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_exposure_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_exposure_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_exposure_time);
      union {
        float real;
        uint32_t base;
      } u_reached_gain_value;
      u_reached_gain_value.real = this->reached_gain_value;
      *(outbuffer + offset + 0) = (u_reached_gain_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reached_gain_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reached_gain_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reached_gain_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reached_gain_value);
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
        int32_t real;
        uint32_t base;
      } u_reached_brightness;
      u_reached_brightness.base = 0;
      u_reached_brightness.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_brightness.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_brightness.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_brightness.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_brightness = u_reached_brightness.real;
      offset += sizeof(this->reached_brightness);
      union {
        float real;
        uint32_t base;
      } u_reached_exposure_time;
      u_reached_exposure_time.base = 0;
      u_reached_exposure_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_exposure_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_exposure_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_exposure_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_exposure_time = u_reached_exposure_time.real;
      offset += sizeof(this->reached_exposure_time);
      union {
        float real;
        uint32_t base;
      } u_reached_gain_value;
      u_reached_gain_value.base = 0;
      u_reached_gain_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reached_gain_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reached_gain_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reached_gain_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->reached_gain_value = u_reached_gain_value.real;
      offset += sizeof(this->reached_gain_value);
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

    virtual const char * getType() override { return SETBRIGHTNESS; };
    virtual const char * getMD5() override { return "62110aff39d46cf6a4ab77f09896582b"; };

  };

  class SetBrightness {
    public:
    typedef SetBrightnessRequest Request;
    typedef SetBrightnessResponse Response;
  };

}
#endif
