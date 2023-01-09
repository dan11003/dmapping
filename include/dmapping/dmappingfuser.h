#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "tf/transform_broadcaster.h"
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_kdl.h"
#include "eigen_conversions/eigen_msg.h"
#include "memory.h"


#include "dmapping/utility.h"
#include "dmapping/dataHandler.h"


namespace dmapping{

class rosbagReader{
public:
  struct Parameters
  {
    std::string imuTopic = "";
    std::string lidarTopic = "";
    std::string bagPath = "";

    void ReadRosParameters(ros::NodeHandle& nh){


      cout << "Load parameters" << endl;
      nh.param<std::string>("/imuTopic", this->imuTopic, "");
      nh.param<std::string>("/lidarTopic", this->lidarTopic, "");
      nh.param<std::string>("/bagPath", this->bagPath, "");
      cout << this->imuTopic << endl;
      cout << this->lidarTopic << endl;
      cout << this->bagPath << endl;
      std::vector<std::string> parNames;
      bool status = nh.getParamNames(parNames);
      for(auto s : parNames)
        cout <<"\t" << s << "\n";
    }
  };

  rosbagReader(ImuHandler& imuHandler, ScanHandler& scanHandler, Parameters& par): imuHandler_(imuHandler), scanHandler_(scanHandler), par_(par) {}

  size_t Read();

private:
  ImuHandler& imuHandler_;
  ScanHandler& scanHandler_;

  rosbag::Bag bag_;
  Parameters par_;

};


class LidarBatch
{
public:
  struct Parameters
  {
    std::string world_frame = "world";
    std::string sensor_frame = "velodyne";
    double roll = 0, pitch = 0, yaw = 3.14;
    void ReadRosParameters(ros::NodeHandle& nh){

      cout << "Load parameters" << endl;
      nh.param<double>("/extrinsicsPitch", this->pitch, 0.0);
      nh.param<double>("/extrinsicsRoll", this->roll,  0.0);
      nh.param<double>("/extrinsicsYaw", this->yaw,    0.0);
      cout << this->roll << endl;
      cout << this->pitch << endl;
      cout << this->yaw << endl;
    }
  };
  LidarBatch();

  ImuHandler imuHandler_;
  ScanHandler scanHandler_;
  std::shared_ptr<rosbagReader> reader;


  void Prune();

  void Visualize();

private:

  VectorAffine3d poses_;
  VectorAffine3d measurements_;

  ros::NodeHandle nh;
  Parameters par_;

  Eigen::Quaterniond extrinsicsLid2imu;

};

class Fuser
{
public:
  Fuser(LidarBatch& batch) : batch_(batch) {}

  std::vector<Eigen::Affine3d> trajectory_;

  LidarBatch& batch_;
};

}
