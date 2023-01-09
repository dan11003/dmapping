#pragma once
#ifndef VELODYNEPOINT_H
#define VELODYNEPOINT_H
#define PCL_NO_PRECOMPILE


#include <pcl/point_types.h>

namespace dmapping
{
struct PointXYZIRT
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float         intensity;            ///< laser intensity reading
  std::uint16_t ring;                 ///< laser ring number
  float         time;                 ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
}
EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(dmapping::PointXYZIRT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time))

#endif  // VELODYNE_PCL_POINT_TYPES_H
