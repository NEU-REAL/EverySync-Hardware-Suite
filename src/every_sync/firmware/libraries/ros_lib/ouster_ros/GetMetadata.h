#ifndef _ROS_SERVICE_GetMetadata_h
#define _ROS_SERVICE_GetMetadata_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ouster_ros
{

static const char GETMETADATA[] = "ouster_ros/GetMetadata";

  class GetMetadataRequest : public ros::Msg
  {
    public:

    GetMetadataRequest()
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

    virtual const char * getType() override { return GETMETADATA; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetMetadataResponse : public ros::Msg
  {
    public:
      typedef const char* _metadata_type;
      _metadata_type metadata;

    GetMetadataResponse():
      metadata("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_metadata = strlen(this->metadata);
      varToArr(outbuffer + offset, length_metadata);
      offset += 4;
      memcpy(outbuffer + offset, this->metadata, length_metadata);
      offset += length_metadata;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_metadata;
      arrToVar(length_metadata, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_metadata; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_metadata-1]=0;
      this->metadata = (char *)(inbuffer + offset-1);
      offset += length_metadata;
     return offset;
    }

    virtual const char * getType() override { return GETMETADATA; };
    virtual const char * getMD5() override { return "d37888e5a47bef783c189dec5351027e"; };

  };

  class GetMetadata {
    public:
    typedef GetMetadataRequest Request;
    typedef GetMetadataResponse Response;
  };

}
#endif
