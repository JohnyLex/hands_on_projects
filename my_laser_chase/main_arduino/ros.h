#ifndef _ROS_H_
#define _ROS_H_

#include <ros_empty.h>
#include "ros/node_handle.h"
#include "ArduinoHardware.h"

namespace ros
{
  // default is 25, 25, 512, 512  
  //typedef NodeHandle_<ArduinoHardware, 10, 15, 128, 256> NodeHandle;
  //10 Publishers, 15 Subscriber, 128 bytes for input buffer and 256 bytes for output buffer
  
  typedef NodeHandle_<ArduinoHardware, 1, 1, 128, 128> NodeHandle;

  // This is legal too and will use the default 25, 25, 512, 512
  //typedef NodeHandle_<ArduinoHardware> NodeHandle;
}

#endif
