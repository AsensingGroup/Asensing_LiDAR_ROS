
#pragma once

#include "ag_driver/msg/point_cloud_msg.hpp"

#ifdef POINT_TYPE_XYZIRT
typedef PointCloudT<PointXYZIRT> LidarPointCloudMsg;
#else
typedef PointCloudT<PointXYZI> LidarPointCloudMsg;
#endif

