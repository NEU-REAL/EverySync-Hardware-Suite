#ifndef _ROS_realsense2_camera_IMUInfo_h
#define _ROS_realsense2_camera_IMUInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense2_camera
{

  class IMUInfo : public ros::Msg
  {
    public:
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;
      float data[12];
      float noise_variances[3];
      float bias_variances[3];

    IMUInfo():
      frame_id(""),
      data(),
      noise_variances(),
      bias_variances()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      for( uint32_t i = 0; i < 12; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->noise_variances[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->bias_variances[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
      for( uint32_t i = 0; i < 12; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->data[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->noise_variances[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->bias_variances[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "realsense2_camera/IMUInfo"; };
    virtual const char * getMD5() override { return "a02adb3a99530b11ba18a16f40f9512a"; };

  };

}
#endif
