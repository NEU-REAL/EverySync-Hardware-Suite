#ifndef _ROS_SERVICE_GetConfig_h
#define _ROS_SERVICE_GetConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ouster_ros
{

static const char GETCONFIG[] = "ouster_ros/GetConfig";

  class GetConfigRequest : public ros::Msg
  {
    public:

    GetConfigRequest()
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

    virtual const char * getType() override { return GETCONFIG; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetConfigResponse : public ros::Msg
  {
    public:
      typedef const char* _config_type;
      _config_type config;

    GetConfigResponse():
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

    virtual const char * getType() override { return GETCONFIG; };
    virtual const char * getMD5() override { return "b3532af339db184b4a6a974d00ee4fe6"; };

  };

  class GetConfig {
    public:
    typedef GetConfigRequest Request;
    typedef GetConfigResponse Response;
  };

}
#endif
