#ifndef _ROS_mech_ros_msgs_MarkerPose_h
#define _ROS_mech_ros_msgs_MarkerPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "mech_ros_msgs/OrientationRPY.h"

namespace mech_ros_msgs
{

  class MarkerPose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef mech_ros_msgs::OrientationRPY _orientation_type;
      _orientation_type orientation;

    MarkerPose():
      position(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mech_ros_msgs/MarkerPose"; };
    const char * getMD5(){ return "aa49ab631228eec32467eb659bcd8cc9"; };

  };

}
#endif