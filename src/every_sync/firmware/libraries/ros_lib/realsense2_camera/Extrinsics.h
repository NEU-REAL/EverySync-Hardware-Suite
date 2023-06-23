#ifndef _ROS_realsense2_camera_Extrinsics_h
#define _ROS_realsense2_camera_Extrinsics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace realsense2_camera
{

  class Extrinsics : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      float rotation[9];
      float translation[3];

    Extrinsics():
      header(),
      rotation(),
      translation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->rotation[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->translation[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->rotation[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->translation[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "realsense2_camera/Extrinsics"; };
    virtual const char * getMD5() override { return "3627b43073f4cd5dd6dc179a49eda2ad"; };

  };

}
#endif
