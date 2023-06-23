#ifndef _ROS_realsense2_camera_Metadata_h
#define _ROS_realsense2_camera_Metadata_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace realsense2_camera
{

  class Metadata : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _json_data_type;
      _json_data_type json_data;

    Metadata():
      header(),
      json_data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_json_data = strlen(this->json_data);
      varToArr(outbuffer + offset, length_json_data);
      offset += 4;
      memcpy(outbuffer + offset, this->json_data, length_json_data);
      offset += length_json_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_json_data;
      arrToVar(length_json_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_json_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_json_data-1]=0;
      this->json_data = (char *)(inbuffer + offset-1);
      offset += length_json_data;
     return offset;
    }

    virtual const char * getType() override { return "realsense2_camera/Metadata"; };
    virtual const char * getMD5() override { return "4966ca002be16ee67fe4dbfb2f354787"; };

  };

}
#endif
