#ifndef _ROS_plotjuggler_msgs_DataPoint_h
#define _ROS_plotjuggler_msgs_DataPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace plotjuggler_msgs
{

  class DataPoint : public ros::Msg
  {
    public:
      typedef uint16_t _name_index_type;
      _name_index_type name_index;
      typedef float _stamp_type;
      _stamp_type stamp;
      typedef float _value_type;
      _value_type value;

    DataPoint():
      name_index(0),
      stamp(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->name_index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_index >> (8 * 1)) & 0xFF;
      offset += sizeof(this->name_index);
      offset += serializeAvrFloat64(outbuffer + offset, this->stamp);
      offset += serializeAvrFloat64(outbuffer + offset, this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->name_index =  ((uint16_t) (*(inbuffer + offset)));
      this->name_index |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->name_index);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->stamp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->value));
     return offset;
    }

    virtual const char * getType() override { return "plotjuggler_msgs/DataPoint"; };
    virtual const char * getMD5() override { return "580ca7c40f92b9a6ab4b921c02ebcd28"; };

  };

}
#endif
