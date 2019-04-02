#ifndef _ROS_mech_ros_msgs_RawImu_h
#define _ROS_mech_ros_msgs_RawImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace mech_ros_msgs
{

  class RawImu : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _raw_linear_acceleration_type;
      _raw_linear_acceleration_type raw_linear_acceleration;
      typedef geometry_msgs::Vector3 _raw_angular_velocity_type;
      _raw_angular_velocity_type raw_angular_velocity;
      typedef geometry_msgs::Vector3 _raw_magnetic_field_type;
      _raw_magnetic_field_type raw_magnetic_field;

    RawImu():
      raw_linear_acceleration(),
      raw_angular_velocity(),
      raw_magnetic_field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->raw_linear_acceleration.serialize(outbuffer + offset);
      offset += this->raw_angular_velocity.serialize(outbuffer + offset);
      offset += this->raw_magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->raw_linear_acceleration.deserialize(inbuffer + offset);
      offset += this->raw_angular_velocity.deserialize(inbuffer + offset);
      offset += this->raw_magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mech_ros_msgs/RawImu"; };
    const char * getMD5(){ return "d9a9309a55804015265d47224b58cf1c"; };

  };

}
#endif