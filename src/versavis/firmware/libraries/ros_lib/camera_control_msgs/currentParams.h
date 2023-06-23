#ifndef _ROS_camera_control_msgs_currentParams_h
#define _ROS_camera_control_msgs_currentParams_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace camera_control_msgs
{

  class currentParams : public ros::Msg
  {
    public:
      typedef uint32_t _offset_x_type;
      _offset_x_type offset_x;
      typedef uint32_t _offset_y_type;
      _offset_y_type offset_y;
      typedef bool _reverse_x_type;
      _reverse_x_type reverse_x;
      typedef bool _reverse_y_type;
      _reverse_y_type reverse_y;
      typedef int32_t _black_level_type;
      _black_level_type black_level;
      typedef int32_t _pgi_mode_type;
      _pgi_mode_type pgi_mode;
      typedef int32_t _demosaicing_mode_type;
      _demosaicing_mode_type demosaicing_mode;
      typedef float _noise_reduction_type;
      _noise_reduction_type noise_reduction;
      typedef float _sharpness_enhancement_type;
      _sharpness_enhancement_type sharpness_enhancement;
      typedef int32_t _light_source_preset_type;
      _light_source_preset_type light_source_preset;
      typedef int32_t _balance_white_auto_type;
      _balance_white_auto_type balance_white_auto;
      typedef int32_t _sensor_readout_mode_type;
      _sensor_readout_mode_type sensor_readout_mode;
      typedef int32_t _acquisition_frame_count_type;
      _acquisition_frame_count_type acquisition_frame_count;
      typedef int32_t _trigger_selector_type;
      _trigger_selector_type trigger_selector;
      typedef int32_t _trigger_mode_type;
      _trigger_mode_type trigger_mode;
      typedef int32_t _trigger_source_type;
      _trigger_source_type trigger_source;
      typedef int32_t _trigger_activation_type;
      _trigger_activation_type trigger_activation;
      typedef float _trigger_delay_type;
      _trigger_delay_type trigger_delay;
      typedef int32_t _user_set_selector_type;
      _user_set_selector_type user_set_selector;
      typedef int32_t _user_set_default_selector_type;
      _user_set_default_selector_type user_set_default_selector;
      typedef bool _is_sleeping_type;
      _is_sleeping_type is_sleeping;
      typedef float _brightness_type;
      _brightness_type brightness;
      typedef float _exposure_type;
      _exposure_type exposure;
      typedef float _gain_type;
      _gain_type gain;
      typedef float _gamma_type;
      _gamma_type gamma;
      typedef uint32_t _binning_x_type;
      _binning_x_type binning_x;
      typedef uint32_t _binning_y_type;
      _binning_y_type binning_y;
      typedef int32_t _MaxNumBuffer_type;
      _MaxNumBuffer_type MaxNumBuffer;
      typedef sensor_msgs::RegionOfInterest _roi_type;
      _roi_type roi;
      uint32_t available_image_encoding_length;
      typedef char* _available_image_encoding_type;
      _available_image_encoding_type st_available_image_encoding;
      _available_image_encoding_type * available_image_encoding;
      typedef const char* _current_image_encoding_type;
      _current_image_encoding_type current_image_encoding;
      typedef const char* _current_image_ros_encoding_type;
      _current_image_ros_encoding_type current_image_ros_encoding;
      typedef bool _sucess_type;
      _sucess_type sucess;
      typedef const char* _message_type;
      _message_type message;
      typedef float _temperature_type;
      _temperature_type temperature;

    currentParams():
      offset_x(0),
      offset_y(0),
      reverse_x(0),
      reverse_y(0),
      black_level(0),
      pgi_mode(0),
      demosaicing_mode(0),
      noise_reduction(0),
      sharpness_enhancement(0),
      light_source_preset(0),
      balance_white_auto(0),
      sensor_readout_mode(0),
      acquisition_frame_count(0),
      trigger_selector(0),
      trigger_mode(0),
      trigger_source(0),
      trigger_activation(0),
      trigger_delay(0),
      user_set_selector(0),
      user_set_default_selector(0),
      is_sleeping(0),
      brightness(0),
      exposure(0),
      gain(0),
      gamma(0),
      binning_x(0),
      binning_y(0),
      MaxNumBuffer(0),
      roi(),
      available_image_encoding_length(0), st_available_image_encoding(), available_image_encoding(nullptr),
      current_image_encoding(""),
      current_image_ros_encoding(""),
      sucess(0),
      message(""),
      temperature(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->offset_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->offset_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->offset_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->offset_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offset_x);
      *(outbuffer + offset + 0) = (this->offset_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->offset_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->offset_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->offset_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offset_y);
      union {
        bool real;
        uint8_t base;
      } u_reverse_x;
      u_reverse_x.real = this->reverse_x;
      *(outbuffer + offset + 0) = (u_reverse_x.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reverse_x);
      union {
        bool real;
        uint8_t base;
      } u_reverse_y;
      u_reverse_y.real = this->reverse_y;
      *(outbuffer + offset + 0) = (u_reverse_y.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reverse_y);
      union {
        int32_t real;
        uint32_t base;
      } u_black_level;
      u_black_level.real = this->black_level;
      *(outbuffer + offset + 0) = (u_black_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_black_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_black_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_black_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->black_level);
      union {
        int32_t real;
        uint32_t base;
      } u_pgi_mode;
      u_pgi_mode.real = this->pgi_mode;
      *(outbuffer + offset + 0) = (u_pgi_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pgi_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pgi_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pgi_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pgi_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_demosaicing_mode;
      u_demosaicing_mode.real = this->demosaicing_mode;
      *(outbuffer + offset + 0) = (u_demosaicing_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_demosaicing_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_demosaicing_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_demosaicing_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->demosaicing_mode);
      union {
        float real;
        uint32_t base;
      } u_noise_reduction;
      u_noise_reduction.real = this->noise_reduction;
      *(outbuffer + offset + 0) = (u_noise_reduction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_noise_reduction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_noise_reduction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_noise_reduction.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->noise_reduction);
      union {
        float real;
        uint32_t base;
      } u_sharpness_enhancement;
      u_sharpness_enhancement.real = this->sharpness_enhancement;
      *(outbuffer + offset + 0) = (u_sharpness_enhancement.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sharpness_enhancement.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sharpness_enhancement.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sharpness_enhancement.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sharpness_enhancement);
      union {
        int32_t real;
        uint32_t base;
      } u_light_source_preset;
      u_light_source_preset.real = this->light_source_preset;
      *(outbuffer + offset + 0) = (u_light_source_preset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_light_source_preset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_light_source_preset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_light_source_preset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->light_source_preset);
      union {
        int32_t real;
        uint32_t base;
      } u_balance_white_auto;
      u_balance_white_auto.real = this->balance_white_auto;
      *(outbuffer + offset + 0) = (u_balance_white_auto.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_balance_white_auto.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_balance_white_auto.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_balance_white_auto.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->balance_white_auto);
      union {
        int32_t real;
        uint32_t base;
      } u_sensor_readout_mode;
      u_sensor_readout_mode.real = this->sensor_readout_mode;
      *(outbuffer + offset + 0) = (u_sensor_readout_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sensor_readout_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sensor_readout_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sensor_readout_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_readout_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_acquisition_frame_count;
      u_acquisition_frame_count.real = this->acquisition_frame_count;
      *(outbuffer + offset + 0) = (u_acquisition_frame_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acquisition_frame_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acquisition_frame_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acquisition_frame_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acquisition_frame_count);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_selector;
      u_trigger_selector.real = this->trigger_selector;
      *(outbuffer + offset + 0) = (u_trigger_selector.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trigger_selector.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trigger_selector.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trigger_selector.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_selector);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_mode;
      u_trigger_mode.real = this->trigger_mode;
      *(outbuffer + offset + 0) = (u_trigger_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trigger_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trigger_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trigger_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_source;
      u_trigger_source.real = this->trigger_source;
      *(outbuffer + offset + 0) = (u_trigger_source.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trigger_source.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trigger_source.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trigger_source.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_source);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_activation;
      u_trigger_activation.real = this->trigger_activation;
      *(outbuffer + offset + 0) = (u_trigger_activation.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trigger_activation.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trigger_activation.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trigger_activation.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_activation);
      union {
        float real;
        uint32_t base;
      } u_trigger_delay;
      u_trigger_delay.real = this->trigger_delay;
      *(outbuffer + offset + 0) = (u_trigger_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trigger_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trigger_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trigger_delay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_delay);
      union {
        int32_t real;
        uint32_t base;
      } u_user_set_selector;
      u_user_set_selector.real = this->user_set_selector;
      *(outbuffer + offset + 0) = (u_user_set_selector.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_set_selector.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_set_selector.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_set_selector.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_set_selector);
      union {
        int32_t real;
        uint32_t base;
      } u_user_set_default_selector;
      u_user_set_default_selector.real = this->user_set_default_selector;
      *(outbuffer + offset + 0) = (u_user_set_default_selector.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_set_default_selector.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_set_default_selector.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_set_default_selector.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_set_default_selector);
      union {
        bool real;
        uint8_t base;
      } u_is_sleeping;
      u_is_sleeping.real = this->is_sleeping;
      *(outbuffer + offset + 0) = (u_is_sleeping.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_sleeping);
      union {
        float real;
        uint32_t base;
      } u_brightness;
      u_brightness.real = this->brightness;
      *(outbuffer + offset + 0) = (u_brightness.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brightness.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_brightness.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_brightness.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->brightness);
      union {
        float real;
        uint32_t base;
      } u_exposure;
      u_exposure.real = this->exposure;
      *(outbuffer + offset + 0) = (u_exposure.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_exposure.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_exposure.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_exposure.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->exposure);
      union {
        float real;
        uint32_t base;
      } u_gain;
      u_gain.real = this->gain;
      *(outbuffer + offset + 0) = (u_gain.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gain.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gain.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gain.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gain);
      union {
        float real;
        uint32_t base;
      } u_gamma;
      u_gamma.real = this->gamma;
      *(outbuffer + offset + 0) = (u_gamma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gamma.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gamma.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gamma.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gamma);
      *(outbuffer + offset + 0) = (this->binning_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_x);
      *(outbuffer + offset + 0) = (this->binning_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_MaxNumBuffer;
      u_MaxNumBuffer.real = this->MaxNumBuffer;
      *(outbuffer + offset + 0) = (u_MaxNumBuffer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_MaxNumBuffer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_MaxNumBuffer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_MaxNumBuffer.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->MaxNumBuffer);
      offset += this->roi.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->available_image_encoding_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->available_image_encoding_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->available_image_encoding_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->available_image_encoding_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->available_image_encoding_length);
      for( uint32_t i = 0; i < available_image_encoding_length; i++){
      uint32_t length_available_image_encodingi = strlen(this->available_image_encoding[i]);
      varToArr(outbuffer + offset, length_available_image_encodingi);
      offset += 4;
      memcpy(outbuffer + offset, this->available_image_encoding[i], length_available_image_encodingi);
      offset += length_available_image_encodingi;
      }
      uint32_t length_current_image_encoding = strlen(this->current_image_encoding);
      varToArr(outbuffer + offset, length_current_image_encoding);
      offset += 4;
      memcpy(outbuffer + offset, this->current_image_encoding, length_current_image_encoding);
      offset += length_current_image_encoding;
      uint32_t length_current_image_ros_encoding = strlen(this->current_image_ros_encoding);
      varToArr(outbuffer + offset, length_current_image_ros_encoding);
      offset += 4;
      memcpy(outbuffer + offset, this->current_image_ros_encoding, length_current_image_ros_encoding);
      offset += length_current_image_ros_encoding;
      union {
        bool real;
        uint8_t base;
      } u_sucess;
      u_sucess.real = this->sucess;
      *(outbuffer + offset + 0) = (u_sucess.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sucess);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->offset_x =  ((uint32_t) (*(inbuffer + offset)));
      this->offset_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->offset_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->offset_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->offset_x);
      this->offset_y =  ((uint32_t) (*(inbuffer + offset)));
      this->offset_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->offset_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->offset_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->offset_y);
      union {
        bool real;
        uint8_t base;
      } u_reverse_x;
      u_reverse_x.base = 0;
      u_reverse_x.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reverse_x = u_reverse_x.real;
      offset += sizeof(this->reverse_x);
      union {
        bool real;
        uint8_t base;
      } u_reverse_y;
      u_reverse_y.base = 0;
      u_reverse_y.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reverse_y = u_reverse_y.real;
      offset += sizeof(this->reverse_y);
      union {
        int32_t real;
        uint32_t base;
      } u_black_level;
      u_black_level.base = 0;
      u_black_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_black_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_black_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_black_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->black_level = u_black_level.real;
      offset += sizeof(this->black_level);
      union {
        int32_t real;
        uint32_t base;
      } u_pgi_mode;
      u_pgi_mode.base = 0;
      u_pgi_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pgi_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pgi_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pgi_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pgi_mode = u_pgi_mode.real;
      offset += sizeof(this->pgi_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_demosaicing_mode;
      u_demosaicing_mode.base = 0;
      u_demosaicing_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_demosaicing_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_demosaicing_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_demosaicing_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->demosaicing_mode = u_demosaicing_mode.real;
      offset += sizeof(this->demosaicing_mode);
      union {
        float real;
        uint32_t base;
      } u_noise_reduction;
      u_noise_reduction.base = 0;
      u_noise_reduction.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_noise_reduction.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_noise_reduction.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_noise_reduction.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->noise_reduction = u_noise_reduction.real;
      offset += sizeof(this->noise_reduction);
      union {
        float real;
        uint32_t base;
      } u_sharpness_enhancement;
      u_sharpness_enhancement.base = 0;
      u_sharpness_enhancement.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sharpness_enhancement.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sharpness_enhancement.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sharpness_enhancement.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sharpness_enhancement = u_sharpness_enhancement.real;
      offset += sizeof(this->sharpness_enhancement);
      union {
        int32_t real;
        uint32_t base;
      } u_light_source_preset;
      u_light_source_preset.base = 0;
      u_light_source_preset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_light_source_preset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_light_source_preset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_light_source_preset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->light_source_preset = u_light_source_preset.real;
      offset += sizeof(this->light_source_preset);
      union {
        int32_t real;
        uint32_t base;
      } u_balance_white_auto;
      u_balance_white_auto.base = 0;
      u_balance_white_auto.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_balance_white_auto.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_balance_white_auto.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_balance_white_auto.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->balance_white_auto = u_balance_white_auto.real;
      offset += sizeof(this->balance_white_auto);
      union {
        int32_t real;
        uint32_t base;
      } u_sensor_readout_mode;
      u_sensor_readout_mode.base = 0;
      u_sensor_readout_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sensor_readout_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sensor_readout_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sensor_readout_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sensor_readout_mode = u_sensor_readout_mode.real;
      offset += sizeof(this->sensor_readout_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_acquisition_frame_count;
      u_acquisition_frame_count.base = 0;
      u_acquisition_frame_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acquisition_frame_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acquisition_frame_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acquisition_frame_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acquisition_frame_count = u_acquisition_frame_count.real;
      offset += sizeof(this->acquisition_frame_count);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_selector;
      u_trigger_selector.base = 0;
      u_trigger_selector.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trigger_selector.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trigger_selector.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trigger_selector.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trigger_selector = u_trigger_selector.real;
      offset += sizeof(this->trigger_selector);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_mode;
      u_trigger_mode.base = 0;
      u_trigger_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trigger_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trigger_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trigger_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trigger_mode = u_trigger_mode.real;
      offset += sizeof(this->trigger_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_source;
      u_trigger_source.base = 0;
      u_trigger_source.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trigger_source.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trigger_source.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trigger_source.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trigger_source = u_trigger_source.real;
      offset += sizeof(this->trigger_source);
      union {
        int32_t real;
        uint32_t base;
      } u_trigger_activation;
      u_trigger_activation.base = 0;
      u_trigger_activation.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trigger_activation.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trigger_activation.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trigger_activation.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trigger_activation = u_trigger_activation.real;
      offset += sizeof(this->trigger_activation);
      union {
        float real;
        uint32_t base;
      } u_trigger_delay;
      u_trigger_delay.base = 0;
      u_trigger_delay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trigger_delay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trigger_delay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trigger_delay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->trigger_delay = u_trigger_delay.real;
      offset += sizeof(this->trigger_delay);
      union {
        int32_t real;
        uint32_t base;
      } u_user_set_selector;
      u_user_set_selector.base = 0;
      u_user_set_selector.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_set_selector.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_set_selector.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_set_selector.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_set_selector = u_user_set_selector.real;
      offset += sizeof(this->user_set_selector);
      union {
        int32_t real;
        uint32_t base;
      } u_user_set_default_selector;
      u_user_set_default_selector.base = 0;
      u_user_set_default_selector.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_set_default_selector.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_set_default_selector.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_set_default_selector.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_set_default_selector = u_user_set_default_selector.real;
      offset += sizeof(this->user_set_default_selector);
      union {
        bool real;
        uint8_t base;
      } u_is_sleeping;
      u_is_sleeping.base = 0;
      u_is_sleeping.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_sleeping = u_is_sleeping.real;
      offset += sizeof(this->is_sleeping);
      union {
        float real;
        uint32_t base;
      } u_brightness;
      u_brightness.base = 0;
      u_brightness.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brightness.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_brightness.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_brightness.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->brightness = u_brightness.real;
      offset += sizeof(this->brightness);
      union {
        float real;
        uint32_t base;
      } u_exposure;
      u_exposure.base = 0;
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_exposure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->exposure = u_exposure.real;
      offset += sizeof(this->exposure);
      union {
        float real;
        uint32_t base;
      } u_gain;
      u_gain.base = 0;
      u_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gain = u_gain.real;
      offset += sizeof(this->gain);
      union {
        float real;
        uint32_t base;
      } u_gamma;
      u_gamma.base = 0;
      u_gamma.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gamma.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gamma.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gamma.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gamma = u_gamma.real;
      offset += sizeof(this->gamma);
      this->binning_x =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_x);
      this->binning_y =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_y);
      union {
        int32_t real;
        uint32_t base;
      } u_MaxNumBuffer;
      u_MaxNumBuffer.base = 0;
      u_MaxNumBuffer.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_MaxNumBuffer.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_MaxNumBuffer.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_MaxNumBuffer.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->MaxNumBuffer = u_MaxNumBuffer.real;
      offset += sizeof(this->MaxNumBuffer);
      offset += this->roi.deserialize(inbuffer + offset);
      uint32_t available_image_encoding_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      available_image_encoding_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      available_image_encoding_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      available_image_encoding_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->available_image_encoding_length);
      if(available_image_encoding_lengthT > available_image_encoding_length)
        this->available_image_encoding = (char**)realloc(this->available_image_encoding, available_image_encoding_lengthT * sizeof(char*));
      available_image_encoding_length = available_image_encoding_lengthT;
      for( uint32_t i = 0; i < available_image_encoding_length; i++){
      uint32_t length_st_available_image_encoding;
      arrToVar(length_st_available_image_encoding, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_available_image_encoding; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_available_image_encoding-1]=0;
      this->st_available_image_encoding = (char *)(inbuffer + offset-1);
      offset += length_st_available_image_encoding;
        memcpy( &(this->available_image_encoding[i]), &(this->st_available_image_encoding), sizeof(char*));
      }
      uint32_t length_current_image_encoding;
      arrToVar(length_current_image_encoding, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_image_encoding; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_image_encoding-1]=0;
      this->current_image_encoding = (char *)(inbuffer + offset-1);
      offset += length_current_image_encoding;
      uint32_t length_current_image_ros_encoding;
      arrToVar(length_current_image_ros_encoding, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_image_ros_encoding; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_image_ros_encoding-1]=0;
      this->current_image_ros_encoding = (char *)(inbuffer + offset-1);
      offset += length_current_image_ros_encoding;
      union {
        bool real;
        uint8_t base;
      } u_sucess;
      u_sucess.base = 0;
      u_sucess.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sucess = u_sucess.real;
      offset += sizeof(this->sucess);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
     return offset;
    }

    virtual const char * getType() override { return "camera_control_msgs/currentParams"; };
    virtual const char * getMD5() override { return "442a674eef3748e9fe89b83004286960"; };

  };

}
#endif
