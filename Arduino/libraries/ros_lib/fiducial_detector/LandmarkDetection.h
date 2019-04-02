#ifndef _ROS_fiducial_detector_LandmarkDetection_h
#define _ROS_fiducial_detector_LandmarkDetection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

namespace fiducial_detector
{

  class LandmarkDetection : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseWithCovarianceStamped _detected_pose_type;
      _detected_pose_type detected_pose;
      typedef geometry_msgs::PoseStamped _expected_pose_type;
      _expected_pose_type expected_pose;

    LandmarkDetection():
      detected_pose(),
      expected_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->detected_pose.serialize(outbuffer + offset);
      offset += this->expected_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->detected_pose.deserialize(inbuffer + offset);
      offset += this->expected_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fiducial_detector/LandmarkDetection"; };
    const char * getMD5(){ return "9a01022571340f8586bf90437503a5f0"; };

  };

}
#endif