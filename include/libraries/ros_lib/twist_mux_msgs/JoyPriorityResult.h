#ifndef _ROS_twist_mux_msgs_JoyPriorityResult_h
#define _ROS_twist_mux_msgs_JoyPriorityResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace twist_mux_msgs
{

  class JoyPriorityResult : public ros::Msg
  {
    public:

    JoyPriorityResult()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "twist_mux_msgs/JoyPriorityResult"; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

}
#endif