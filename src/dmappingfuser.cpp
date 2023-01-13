#include "dmapping/dmappingfuser.h"
namespace dmapping {

bool Compensate(RingCloud::Ptr input, RingCloud::Ptr compensated, ImuHandler& handler, Eigen::Quaterniond& extrinsics){

  compensated->resize(input->size());
  const double tScan = pcl_conversions::fromPCL(input->header.stamp).toSec();
  const double t0 = input->points.front().time + tScan;
  const double t1 = input->points.back().time + tScan;
  if(!handler.TimeContained(t0) || !handler.TimeContained(t1) ){
    std::cout << "no imu data" << std::endl;
    return false;
  }

  std::cout << "begin: " <<  t0 << ", end: " << t1<< ", scan:" << GetRelTime(tScan) << std::endl;
  const Eigen::Quaterniond qInit(Imu2Orientation(handler.Get(tScan))*extrinsics);
  const Eigen::Quaterniond qInitInv = qInit.inverse();

  std::cout << "qInit" << qInit.matrix() << std::endl;
  ros::Time tr0 = ros::Time::now();
  for(int i = 0 ; i <input->points.size() ; i++){
    const double timeCurrent = tScan + input->points[i].time;
    const Eigen::Quaterniond qNow = Imu2Orientation(handler.Get(timeCurrent))*extrinsics;
    const Eigen::Quaterniond qDiff = qInitInv*qNow;
    const Eigen::Vector3d pTransformed = qDiff*Eigen::Vector3d(input->points[i].x, input->points[i].y, input->points[i].z);
    compensated->points[i].x = pTransformed(0); compensated->points[i].y = pTransformed(1); compensated->points[i].z = pTransformed(2);
  }
  ros::Time tr1 = ros::Time::now();
  cout << "time elapse: " << tr1-tr0 << endl;
  return true;

}

size_t rosbagReader::Read(){
  std::cout << "open bag file: " <<  par_.bagPath << std::endl;
  bag_.open(par_.bagPath, rosbag::bagmode::Read);

  const std::vector<std::string> topics{ par_.imuTopic, par_.lidarTopic};
  rosbag::View view(bag_, rosbag::TopicQuery(topics));
  int counter = 0;

  for (rosbag::MessageInstance const m : view) {
    if(!ros::ok()){
      return size_t(0);
    }
    if(counter == 0){
      boost::ignore_unused(GetRelTime(m.getTime().toSec()));
    }
    const double progress = 100*(double)counter++ /((double)view.size());
    ROS_INFO_STREAM_THROTTLE(1, "\nRead data" << progress << " %\n" );

    if( m.getTopic() == par_.lidarTopic){
      if(sensor_msgs::PointCloud2::ConstPtr scan_msg = m.instantiate<sensor_msgs::PointCloud2>()){
        scanHandler_.AddMsg(scan_msg);
      }
    }
    else if(sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>()){
      imuHandler_.AddMsg(imu_msg);
    }
  }
  cout << "Load data finished\nIMU: " << std::quoted(par_.imuTopic) << ": " << imuHandler_.size() << "\nLidar: " << std::quoted( par_.lidarTopic) <<": " << scanHandler_.size() << endl;
  return std::min(imuHandler_.size(), scanHandler_.size());

}



LidarBatch::LidarBatch() : nh("~"){

  rosbagReader::Parameters readerPar;
  readerPar.ReadRosParameters(nh);
  par_.ReadRosParameters(nh);
  extrinsicsLid2imu = euler2Quaternion(par_.roll, par_.pitch, par_.yaw);

  reader = std::make_shared<rosbagReader>(imuHandler_,scanHandler_, readerPar);
  reader->Read();
  Prune();
  const Eigen::Affine3d identity = Eigen::Affine3d::Identity();
  poses_.resize(scanHandler_.size(), identity);
  measurements_.resize(scanHandler_.size(), identity);


  for(auto itr = scanHandler_.begin() ; itr != scanHandler_.end() ; itr++){
    const double tCurrent = itr->GetStamp();
    //cout << "tfirst : "  << tFirst << endl;
    //cout << "tcurrent: "  << tCurrent << endl;
    //cout  << GetRelTime(tCurrent) << ", ";

    Eigen::Quaterniond rot = Imu2Orientation(imuHandler_.Get(tCurrent));

    Eigen::Quaterniond rotTransformed = rot*extrinsicsLid2imu;
    rotTransformed.normalize();
    const Eigen::Affine3d affineRot(rotTransformed);
    const size_t idx = std::distance(scanHandler_.begin(),itr);
    poses_[idx] = measurements_[idx] = affineRot;
    //cout << affineRot.linear() << endl;
  }
  //cout << "all scans" << endl;
}

void LidarBatch::PlotAcceleration(const Eigen::Vector3d& acc){
  static ros::NodeHandle nh("~");

  static ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  const double scale = 0.2;
  geometry_msgs::Point p1,p2;
  p1.x = 0; p1.y = 0; p1.z = 0;
  p2.x = scale*acc(0); p2.y = scale*acc(1); p2.z = scale*acc(2);
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  vis_pub.publish( marker );
}
void LidarBatch::Visualize(){
  auto itr_scan = scanHandler_.begin();
  auto itr_imu = imuHandler_.begin();
  while(ros::ok()){
    usleep(1*1000);
    const ros::Time t = ros::Time::now();
    if (itr_imu == imuHandler_.end() || itr_scan == scanHandler_.end()) {
      return;
    }
    if(itr_imu->first < itr_scan->GetStamp()){
      Eigen::Quaterniond rot = Imu2Orientation(itr_imu->second);
      Eigen::Affine3d tIMU(rot*extrinsicsLid2imu);
      //Eigen::Vector3d acc = extrinsicsLid2imu*Imu2LinearAcceleration(itr_imu->second);
      //Eigen::Vector3d angular = extrinsicsLid2imu*Imu2AngularVelocity(itr_imu->second);
      //PlotAcceleration(acc);
      /*const double dt = 0.2;
      Eigen::Quaterniond qdiff = Eigen::AngleAxisd(angular(0)*dt, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(angular(1)*dt, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(angular(2)*dt, Eigen::Vector3d::UnitZ());*/

      PublishTF(par_.world_frame, "imuOdom", tIMU, t);
      //PublishTF("imuOdom", "imuPred", Eigen::Affine3d(qdiff), t);
      itr_imu++;
    }
    else{
      RingCloud::Ptr compensated(new RingCloud());
      if(!Compensate(itr_scan->GetRingCloud(), compensated, imuHandler_, extrinsicsLid2imu)){
        itr_scan++;
        continue;
      }
      const Eigen::Affine3d& pose = poses_[std::distance(scanHandler_.begin(),itr_scan)];
      cout << "idx: " << std::distance(scanHandler_.begin(),itr_scan) << pose.matrix()  << endl;
      PublishTF(par_.world_frame, par_.sensor_frame, pose, t);
      PublishCloud("Scans", *itr_scan->GetCloud(), par_.sensor_frame, t);
      PublishCloud("Compensated", *compensated, par_.sensor_frame, t);
      itr_scan++;
    }
  }
}


void LidarBatch::Prune(){
  while(ros::ok() && scanHandler_.size() > 0){
    if (scanHandler_.begin()->GetStamp() < imuHandler_.begin()->first) {
      cout << "prune first" << endl;
      scanHandler_.data_.erase(scanHandler_.data_.begin());
    }
    else if (scanHandler_.end()->GetStamp() < imuHandler_.end()->first) {
      cout << "prune last" << endl;
      scanHandler_.data_.pop_back();
    }
    else {
      return;
    }
  }
}



}
