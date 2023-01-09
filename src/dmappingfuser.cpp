#include "dmapping/dmappingfuser.h"
namespace dmapping {


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

    Eigen::Quaterniond rot;
    if(!imuHandler_.Get(tCurrent, rot)){
      cout << "PROBLEM TRANSFORM" << endl;
      continue;
    }
    Eigen::Quaterniond rotTransformed = extrinsicsLid2imu*rot;
    rotTransformed.normalize();
    const Eigen::Affine3d affineRot(rotTransformed);
    const size_t idx = std::distance(scanHandler_.begin(),itr);
    poses_[idx] = measurements_[idx] = affineRot;
    //cout << affineRot.linear() << endl;
  }
  //cout << "all scans" << endl;



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
    if(itr_imu->first < itr_scan->stamp_){
      Eigen::Affine3d tIMU(extrinsicsLid2imu*itr_imu->second);
      PublishTF(par_.world_frame, "imuOdom", tIMU, t);
      itr_imu++;
    }
    else{
      const Eigen::Affine3d& pose = poses_[std::distance(scanHandler_.begin(),itr_scan)];
      cout <<"idx: " << std::distance(scanHandler_.begin(),itr_scan) << pose.matrix()  <<endl;
      PublishTF(par_.world_frame, par_.sensor_frame, pose, t);
      PublishCloud("Scans", *itr_scan->GetCloud(), par_.sensor_frame, t);
      itr_scan++;
    }
  }
}


void LidarBatch::Prune(){
  while(ros::ok() && scanHandler_.size() > 0){
    if (scanHandler_.begin()->stamp_ < imuHandler_.begin()->first) {
      cout << "prune first" << endl;
      scanHandler_.data_.erase(scanHandler_.data_.begin());
    }
    else if (scanHandler_.end()->stamp_ < imuHandler_.end()->first) {
      cout << "prune last" << endl;
      scanHandler_.data_.pop_back();
    }
    else {
      return;
    }

  }
}



}
