#ifndef _ROS_ext_clk_msgs_LocalSensorExternalTrigger_h
#define _ROS_ext_clk_msgs_LocalSensorExternalTrigger_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace ext_clk_msgs
{

  class LocalSensorExternalTrigger : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _trigger_id_type;
      _trigger_id_type trigger_id;
      typedef uint32_t _event_id_type;
      _event_id_type event_id;
      typedef ros::Time _timestamp_host_type;
      _timestamp_host_type timestamp_host;

    LocalSensorExternalTrigger():
      header(),
      trigger_id(0),
      event_id(0),
      timestamp_host()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trigger_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trigger_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trigger_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trigger_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trigger_id);
      *(outbuffer + offset + 0) = (this->event_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->event_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->event_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->event_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->event_id);
      *(outbuffer + offset + 0) = (this->timestamp_host.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp_host.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp_host.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp_host.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp_host.sec);
      *(outbuffer + offset + 0) = (this->timestamp_host.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp_host.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp_host.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp_host.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp_host.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->trigger_id =  ((uint32_t) (*(inbuffer + offset)));
      this->trigger_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->trigger_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->trigger_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->trigger_id);
      this->event_id =  ((uint32_t) (*(inbuffer + offset)));
      this->event_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->event_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->event_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->event_id);
      this->timestamp_host.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp_host.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp_host.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp_host.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp_host.sec);
      this->timestamp_host.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp_host.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp_host.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp_host.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp_host.nsec);
     return offset;
    }

    virtual const char * getType() override { return "ext_clk_msgs/LocalSensorExternalTrigger"; };
    virtual const char * getMD5() override { return "a5f284fbc0b54d0c397033867d8565f1"; };

  };

}
#endif
