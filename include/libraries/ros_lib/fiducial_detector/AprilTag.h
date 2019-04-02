#ifndef _ROS_fiducial_detector_AprilTag_h
#define _ROS_fiducial_detector_AprilTag_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace fiducial_detector
{

  class AprilTag : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _id_type;
      _id_type id;
      typedef uint32_t _hamming_distance_type;
      _hamming_distance_type hamming_distance;
      typedef float _distance_type;
      _distance_type distance;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _roll_type;
      _roll_type roll;

    AprilTag():
      header(),
      id(0),
      hamming_distance(0),
      distance(0),
      x(0),
      y(0),
      z(0),
      yaw(0),
      pitch(0),
      roll(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->hamming_distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hamming_distance >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->hamming_distance >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->hamming_distance >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hamming_distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->yaw);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      this->hamming_distance =  ((uint32_t) (*(inbuffer + offset)));
      this->hamming_distance |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hamming_distance |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->hamming_distance |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->hamming_distance);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->yaw));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
     return offset;
    }

    const char * getType(){ return "fiducial_detector/AprilTag"; };
    const char * getMD5(){ return "e9ee45de60fa871dc66fe988ea39e595"; };

  };

}
#endif