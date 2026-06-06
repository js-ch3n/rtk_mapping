// Stub header for livox_ros_driver::CustomMsg
// Provides minimal type definitions needed for compilation when
// livox_ros_driver package is not installed. Only used when
// lidar_type != AVIA (i.e., Velodyne/Ouster mode).
#pragma once

#include <stdint.h>
#include <vector>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace livox_ros_driver {

struct CustomPoint
{
    float x, y, z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;
};

struct CustomMsg
{
    std_msgs::Header header;
    uint64_t timebase;
    uint32_t point_num;
    uint8_t  lidar_id;
    uint8_t  rsvd[3];
    std::vector<CustomPoint> points;

    typedef boost::shared_ptr<CustomMsg> Ptr;
    typedef boost::shared_ptr<CustomMsg const> ConstPtr;
};

} // namespace livox_ros_driver
