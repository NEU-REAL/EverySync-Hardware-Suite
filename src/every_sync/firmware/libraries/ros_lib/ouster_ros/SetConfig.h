#ifndef _ROS_SERVICE_SetConfig_h
#define _ROS_SERVICE_SetConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ouster_ros
{

static const char SETCONFIG[] = "ouster_ros/SetConfig";

  class SetConfigRequest : public ros::Msg
  {
    public:
      typedef const char* _config_file_type;
      _config_file_type config_file;

    SetConfigRequest():
      config_file("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_config_file = strlen(this->config_file);
      varToArr(outbuffer + offset, length_config_file);
      offset += 4;
      memcpy(outbuffer + offset, this->config_file, length_config_file);
      offset += length_config_file;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_config_file;
      arrToVar(length_config_file, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_config_file; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_config_file-1]=0;
      this->config_file = (char *)(inbuffer + offset-1);
      offset += length_config_file;
     return offset;
    }

    virtual const char * getType() override { return SETCONFIG; };
    virtual const char * getMD5() override { return "90949894c75d4db440cc7a08c4bf47dd"; };

  };

  class SetConfigResponse : public ros::Msg
  {
    public:
      typedef const char* _config_type;
      _config_type config;

    SetConfigResponse():
      config("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_config = strlen(this->config);
      varToArr(outbuffer + offset, length_config);
      offset += 4;
      memcpy(outbuffer + offset, this->config, length_config);
      offset += length_config;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_config;
      arrToVar(length_config, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_config; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_config-1]=0;
      this->config = (char *)(inbuffer + offset-1);
      offset += length_config;
     return offset;
    }

    virtual const char * getType() override { return SETCONFIG; };
    virtual const char * getMD5() override { return "b3532af339db184b4a6a974d00ee4fe6"; };

  };

  class SetConfig {
    public:
    typedef SetConfigRequest Request;
    typedef SetConfigResponse Response;
  };

}
#endif
