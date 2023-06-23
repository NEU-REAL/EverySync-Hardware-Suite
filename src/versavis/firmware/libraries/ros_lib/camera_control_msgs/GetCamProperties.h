#ifndef _ROS_SERVICE_GetCamProperties_h
#define _ROS_SERVICE_GetCamProperties_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace camera_control_msgs
{

static const char GETCAMPROPERTIES[] = "camera_control_msgs/GetCamProperties";

  class GetCamPropertiesRequest : public ros::Msg
  {
    public:

    GetCamPropertiesRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return GETCAMPROPERTIES; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetCamPropertiesResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef bool _is_sleeping_type;
      _is_sleeping_type is_sleeping;
      typedef const char* _device_user_id_type;
      _device_user_id_type device_user_id;
      typedef int32_t _min_binning_x_type;
      _min_binning_x_type min_binning_x;
      typedef int32_t _max_binning_x_type;
      _max_binning_x_type max_binning_x;
      typedef int32_t _current_binning_x_type;
      _current_binning_x_type current_binning_x;
      typedef int32_t _min_binning_y_type;
      _min_binning_y_type min_binning_y;
      typedef int32_t _max_binning_y_type;
      _max_binning_y_type max_binning_y;
      typedef int32_t _current_binning_y_type;
      _current_binning_y_type current_binning_y;
      typedef float _max_framerate_type;
      _max_framerate_type max_framerate;
      typedef float _current_framerate_type;
      _current_framerate_type current_framerate;
      typedef float _min_exposure_type;
      _min_exposure_type min_exposure;
      typedef float _max_exposure_type;
      _max_exposure_type max_exposure;
      typedef float _current_exposure_type;
      _current_exposure_type current_exposure;
      typedef float _min_gain_in_cam_units_type;
      _min_gain_in_cam_units_type min_gain_in_cam_units;
      typedef float _max_gain_in_cam_units_type;
      _max_gain_in_cam_units_type max_gain_in_cam_units;
      typedef float _current_gain_in_cam_units_type;
      _current_gain_in_cam_units_type current_gain_in_cam_units;
      typedef float _min_gain_type;
      _min_gain_type min_gain;
      typedef float _max_gain_type;
      _max_gain_type max_gain;
      typedef float _current_gain_type;
      _current_gain_type current_gain;
      typedef float _min_gamma_type;
      _min_gamma_type min_gamma;
      typedef float _max_gamma_type;
      _max_gamma_type max_gamma;
      typedef float _current_gamma_type;
      _current_gamma_type current_gamma;
      typedef bool _brightness_continuous_type;
      _brightness_continuous_type brightness_continuous;
      typedef bool _gain_auto_type;
      _gain_auto_type gain_auto;
      typedef bool _exposure_auto_type;
      _exposure_auto_type exposure_auto;

    GetCamPropertiesResponse():
      success(0),
      is_sleeping(0),
      device_user_id(""),
      min_binning_x(0),
      max_binning_x(0),
      current_binning_x(0),
      min_binning_y(0),
      max_binning_y(0),
      current_binning_y(0),
      max_framerate(0),
      current_framerate(0),
      min_exposure(0),
      max_exposure(0),
      current_exposure(0),
      min_gain_in_cam_units(0),
      max_gain_in_cam_units(0),
      current_gain_in_cam_units(0),
      min_gain(0),
      max_gain(0),
      current_gain(0),
      min_gamma(0),
      max_gamma(0),
      current_gamma(0),
      brightness_continuous(0),
      gain_auto(0),
      exposure_auto(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      union {
        bool real;
        uint8_t base;
      } u_is_sleeping;
      u_is_sleeping.real = this->is_sleeping;
      *(outbuffer + offset + 0) = (u_is_sleeping.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_sleeping);
      uint32_t length_device_user_id = strlen(this->device_user_id);
      varToArr(outbuffer + offset, length_device_user_id);
      offset += 4;
      memcpy(outbuffer + offset, this->device_user_id, length_device_user_id);
      offset += length_device_user_id;
      union {
        int32_t real;
        uint32_t base;
      } u_min_binning_x;
      u_min_binning_x.real = this->min_binning_x;
      *(outbuffer + offset + 0) = (u_min_binning_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_binning_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_binning_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_binning_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_max_binning_x;
      u_max_binning_x.real = this->max_binning_x;
      *(outbuffer + offset + 0) = (u_max_binning_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_binning_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_binning_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_binning_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_current_binning_x;
      u_current_binning_x.real = this->current_binning_x;
      *(outbuffer + offset + 0) = (u_current_binning_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_binning_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_binning_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_binning_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_min_binning_y;
      u_min_binning_y.real = this->min_binning_y;
      *(outbuffer + offset + 0) = (u_min_binning_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_binning_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_binning_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_binning_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_max_binning_y;
      u_max_binning_y.real = this->max_binning_y;
      *(outbuffer + offset + 0) = (u_max_binning_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_binning_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_binning_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_binning_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_current_binning_y;
      u_current_binning_y.real = this->current_binning_y;
      *(outbuffer + offset + 0) = (u_current_binning_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_binning_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_binning_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_binning_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_binning_y);
      union {
        float real;
        uint32_t base;
      } u_max_framerate;
      u_max_framerate.real = this->max_framerate;
      *(outbuffer + offset + 0) = (u_max_framerate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_framerate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_framerate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_framerate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_framerate);
      union {
        float real;
        uint32_t base;
      } u_current_framerate;
      u_current_framerate.real = this->current_framerate;
      *(outbuffer + offset + 0) = (u_current_framerate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_framerate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_framerate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_framerate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_framerate);
      union {
        float real;
        uint32_t base;
      } u_min_exposure;
      u_min_exposure.real = this->min_exposure;
      *(outbuffer + offset + 0) = (u_min_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_exposure);
      union {
        float real;
        uint32_t base;
      } u_max_exposure;
      u_max_exposure.real = this->max_exposure;
      *(outbuffer + offset + 0) = (u_max_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_exposure);
      union {
        float real;
        uint32_t base;
      } u_current_exposure;
      u_current_exposure.real = this->current_exposure;
      *(outbuffer + offset + 0) = (u_current_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_exposure);
      union {
        float real;
        uint32_t base;
      } u_min_gain_in_cam_units;
      u_min_gain_in_cam_units.real = this->min_gain_in_cam_units;
      *(outbuffer + offset + 0) = (u_min_gain_in_cam_units.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_gain_in_cam_units.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_gain_in_cam_units.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_gain_in_cam_units.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_max_gain_in_cam_units;
      u_max_gain_in_cam_units.real = this->max_gain_in_cam_units;
      *(outbuffer + offset + 0) = (u_max_gain_in_cam_units.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_gain_in_cam_units.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_gain_in_cam_units.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_gain_in_cam_units.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_current_gain_in_cam_units;
      u_current_gain_in_cam_units.real = this->current_gain_in_cam_units;
      *(outbuffer + offset + 0) = (u_current_gain_in_cam_units.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_gain_in_cam_units.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_gain_in_cam_units.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_gain_in_cam_units.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_min_gain;
      u_min_gain.real = this->min_gain;
      *(outbuffer + offset + 0) = (u_min_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_gain);
      union {
        float real;
        uint32_t base;
      } u_max_gain;
      u_max_gain.real = this->max_gain;
      *(outbuffer + offset + 0) = (u_max_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_gain);
      union {
        float real;
        uint32_t base;
      } u_current_gain;
      u_current_gain.real = this->current_gain;
      *(outbuffer + offset + 0) = (u_current_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_gain);
      union {
        float real;
        uint32_t base;
      } u_min_gamma;
      u_min_gamma.real = this->min_gamma;
      *(outbuffer + offset + 0) = (u_min_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_gamma);
      union {
        float real;
        uint32_t base;
      } u_max_gamma;
      u_max_gamma.real = this->max_gamma;
      *(outbuffer + offset + 0) = (u_max_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_gamma);
      union {
        float real;
        uint32_t base;
      } u_current_gamma;
      u_current_gamma.real = this->current_gamma;
      *(outbuffer + offset + 0) = (u_current_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_gamma);
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
      } u_gain_auto;
      u_gain_auto.real = this->gain_auto;
      *(outbuffer + offset + 0) = (u_gain_auto.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gain_auto);
      union {
        bool real;
        uint8_t base;
      } u_exposure_auto;
      u_exposure_auto.real = this->exposure_auto;
      *(outbuffer + offset + 0) = (u_exposure_auto.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->exposure_auto);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      union {
        bool real;
        uint8_t base;
      } u_is_sleeping;
      u_is_sleeping.base = 0;
      u_is_sleeping.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_sleeping = u_is_sleeping.real;
      offset += sizeof(this->is_sleeping);
      uint32_t length_device_user_id;
      arrToVar(length_device_user_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_device_user_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_device_user_id-1]=0;
      this->device_user_id = (char *)(inbuffer + offset-1);
      offset += length_device_user_id;
      union {
        int32_t real;
        uint32_t base;
      } u_min_binning_x;
      u_min_binning_x.base = 0;
      u_min_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_binning_x = u_min_binning_x.real;
      offset += sizeof(this->min_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_max_binning_x;
      u_max_binning_x.base = 0;
      u_max_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_binning_x = u_max_binning_x.real;
      offset += sizeof(this->max_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_current_binning_x;
      u_current_binning_x.base = 0;
      u_current_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_binning_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_binning_x = u_current_binning_x.real;
      offset += sizeof(this->current_binning_x);
      union {
        int32_t real;
        uint32_t base;
      } u_min_binning_y;
      u_min_binning_y.base = 0;
      u_min_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_binning_y = u_min_binning_y.real;
      offset += sizeof(this->min_binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_max_binning_y;
      u_max_binning_y.base = 0;
      u_max_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_binning_y = u_max_binning_y.real;
      offset += sizeof(this->max_binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_current_binning_y;
      u_current_binning_y.base = 0;
      u_current_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_binning_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_binning_y = u_current_binning_y.real;
      offset += sizeof(this->current_binning_y);
      union {
        float real;
        uint32_t base;
      } u_max_framerate;
      u_max_framerate.base = 0;
      u_max_framerate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_framerate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_framerate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_framerate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_framerate = u_max_framerate.real;
      offset += sizeof(this->max_framerate);
      union {
        float real;
        uint32_t base;
      } u_current_framerate;
      u_current_framerate.base = 0;
      u_current_framerate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_framerate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_framerate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_framerate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_framerate = u_current_framerate.real;
      offset += sizeof(this->current_framerate);
      union {
        float real;
        uint32_t base;
      } u_min_exposure;
      u_min_exposure.base = 0;
      u_min_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_exposure = u_min_exposure.real;
      offset += sizeof(this->min_exposure);
      union {
        float real;
        uint32_t base;
      } u_max_exposure;
      u_max_exposure.base = 0;
      u_max_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_exposure = u_max_exposure.real;
      offset += sizeof(this->max_exposure);
      union {
        float real;
        uint32_t base;
      } u_current_exposure;
      u_current_exposure.base = 0;
      u_current_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_exposure = u_current_exposure.real;
      offset += sizeof(this->current_exposure);
      union {
        float real;
        uint32_t base;
      } u_min_gain_in_cam_units;
      u_min_gain_in_cam_units.base = 0;
      u_min_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_gain_in_cam_units = u_min_gain_in_cam_units.real;
      offset += sizeof(this->min_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_max_gain_in_cam_units;
      u_max_gain_in_cam_units.base = 0;
      u_max_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_gain_in_cam_units = u_max_gain_in_cam_units.real;
      offset += sizeof(this->max_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_current_gain_in_cam_units;
      u_current_gain_in_cam_units.base = 0;
      u_current_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_gain_in_cam_units.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_gain_in_cam_units = u_current_gain_in_cam_units.real;
      offset += sizeof(this->current_gain_in_cam_units);
      union {
        float real;
        uint32_t base;
      } u_min_gain;
      u_min_gain.base = 0;
      u_min_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_gain = u_min_gain.real;
      offset += sizeof(this->min_gain);
      union {
        float real;
        uint32_t base;
      } u_max_gain;
      u_max_gain.base = 0;
      u_max_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_gain = u_max_gain.real;
      offset += sizeof(this->max_gain);
      union {
        float real;
        uint32_t base;
      } u_current_gain;
      u_current_gain.base = 0;
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_gain = u_current_gain.real;
      offset += sizeof(this->current_gain);
      union {
        float real;
        uint32_t base;
      } u_min_gamma;
      u_min_gamma.base = 0;
      u_min_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_gamma = u_min_gamma.real;
      offset += sizeof(this->min_gamma);
      union {
        float real;
        uint32_t base;
      } u_max_gamma;
      u_max_gamma.base = 0;
      u_max_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_gamma = u_max_gamma.real;
      offset += sizeof(this->max_gamma);
      union {
        float real;
        uint32_t base;
      } u_current_gamma;
      u_current_gamma.base = 0;
      u_current_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_gamma = u_current_gamma.real;
      offset += sizeof(this->current_gamma);
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
      } u_gain_auto;
      u_gain_auto.base = 0;
      u_gain_auto.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gain_auto = u_gain_auto.real;
      offset += sizeof(this->gain_auto);
      union {
        bool real;
        uint8_t base;
      } u_exposure_auto;
      u_exposure_auto.base = 0;
      u_exposure_auto.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->exposure_auto = u_exposure_auto.real;
      offset += sizeof(this->exposure_auto);
     return offset;
    }

    virtual const char * getType() override { return GETCAMPROPERTIES; };
    virtual const char * getMD5() override { return "14bd64331efb0f665787f525f453c05d"; };

  };

  class GetCamProperties {
    public:
    typedef GetCamPropertiesRequest Request;
    typedef GetCamPropertiesResponse Response;
  };

}
#endif
