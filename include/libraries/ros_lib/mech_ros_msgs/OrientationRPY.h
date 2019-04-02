#ifndef _ROS_mech_ros_msgs_OrientationRPY_h
#define _ROS_mech_ros_msgs_OrientationRPY_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mech_ros_msgs
{

  class OrientationRPY : public ros::Msg
  {
    public:
      typedef float _r_type;
      _r_type r;
      typedef float _p_type;
      _p_type p;
      typedef float _y_type;
      _y_type y;

    OrientationRPY():
      r(0),
      p(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      offset += serializeAvrFloat64(outbuffer + offset, this->p);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
     return offset;
    }

    const char * getType(){ return "mech_ros_msgs/OrientationRPY"; };
    const char * getMD5(){ return "c5585e6b09b13f56ed964334fed50470"; };

  };

}
#endif