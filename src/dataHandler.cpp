#include "dmapping/dataHandler.h"

/* A steam of time stamped data  for lookup*/
namespace dmapping {


Eigen::Quaterniond Imu2Orientation(const sensor_msgs::Imu& data){
  return Eigen::Quaterniond(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z);
}

Eigen::Vector3d Imu2AngularVelocity(const sensor_msgs::Imu& data){
  return Eigen::Vector3d(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
}

Eigen::Vector3d Imu2LinearAcceleration(const sensor_msgs::Imu& data){
  return Eigen::Vector3d(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
}

bool compare (const stampedImu i, const stampedImu& j)
{
  return (i.first < j.first);
}

void ImuHandler::AddMsg(sensor_msgs::Imu::ConstPtr msg){
  sensor_msgs::Imu imuMsg = *msg;
  if ( data_.empty()){
    Add(imuMsg);
    return;
  }
  const double tdiff = msg->header.stamp.toSec() - data_.back().first;
  if( tdiff > 0.00001 ){
    Add(imuMsg);
  }
  else{
    //count_invalid++;
    //ROS_INFO_STREAM_THROTTLE(1, "dublicated time stamp, " << count_invalid);
  }
  //cout << "dublicated time stamp, " << count_invalid << endl;;
  //cout << "valid      time stamp, " << count_valid << endl;;
}


void ImuHandler::Add(const sensor_msgs::Imu& data){
  const double tStamp = data.header.stamp.toSec();;
  //tf::quaternionMsgToEigen(data.orientation, orient);
  data_.push_back(std::make_pair(tStamp, data));
}
sensor_msgs::Imu Interpolate(const double tSlerp, const sensor_msgs::Imu& data1, const sensor_msgs::Imu& data2){
  return data1;
}
bool ImuHandler::Get(const double& tStamp, sensor_msgs::Imu& data)const {
  auto first = data_.begin();
  auto last = data_.end();
  stampedImu search = std::make_pair(tStamp,sensor_msgs::Imu());
  auto itr_after = std::lower_bound(first, last, search , compare);
  auto itr_before = std::prev(itr_after, 1);
  if(itr_after != last && itr_after != first && itr_before != first){
    //data = first->second;
    //cout << "elements: " << data_.size() << endl;
    //cout << "search: " << GetRelTime(tStamp) << ", before: "<< GetRelTime(itr_before->first) <<", t diff" <<  itr_before->first - tStamp<<", idx: " << std::distance(data_.begin(), itr_before) << endl;
    //cout << "search: " << tStamp << ", next  : "<< itr_after->first <<", t diff" << itr_after->first - tStamp <<", idx: " << std::distance(data_.begin(), itr_after) << endl;
    const double tSlerp = (tStamp - itr_before->first )/(itr_after->first - itr_before->first);
    data = Interpolate(tSlerp, itr_before->second, itr_after->second);
    //cout << tSlerp << endl;
    //data = itr_before->second.slerp(tSlerp, itr_after->second);
    return true;
  }
  else
    return false;
}
sensor_msgs::Imu ImuHandler::Get(const double& tStamp) const{
  sensor_msgs::Imu data;
  Get(tStamp, data);
  return data;
}
bool ImuHandler::TimeContained(const double t)const{
  if(!data_.empty() && t >= data_.front().first && t <= data_.back().first)
    return true;
  else
    return false;
}

void ScanHandler::AddMsg(sensor_msgs::PointCloud2::ConstPtr laserCloudMsg_c){
  RingCloud::Ptr cloud(new RingCloud);
  sensor_msgs::PointCloud2 laserCloudMsg = *laserCloudMsg_c;
  pcl::moveFromROSMsg(laserCloudMsg, *cloud);
  pcl_conversions::toPCL(laserCloudMsg.header.stamp, cloud->header.stamp);
  Add(cloud);
}

void ScanHandler::Add(RingCloud::Ptr cloud)
{
  data_.push_back(Scan(cloud));
}


/*
template <class T>
void StampedData<T>::Add(const T& val, const double &stamp){
  data_.push_back(std::make_pair(stamp,val));
  if(sizeHistory_ != 0 && data_.size() > sizeHistory_){
    data_.erase(data_.begin());
  }
}
template <class T>
bool StampedData<T>::Get(const double& tStamp, T& data){
  auto first = data_.begin();
  auto last = data_.end();
  if (std::binary_search (first, last, tStamp)){
    cout << "found: idx:" << std::distance(data_.begin(), first) << ", val: " << first->first <<", t diff" << tStamp - first->first << endl;
    data = first->second;
    return true;
  }
  else
    return false;
}

*/
/*

template <class T>
void StampedData<T>::Add(const T& data, const ros::Time& tStamp){
  const sec_t tSec   = tStamp.sec;
  const nsec_t tNsec = tStamp.nsec;
  if(bucket_.find(tSec) == bucket_.end()){
    bucket_[tSec] = batcnNsec();
  }
  bucket_[tSec][tNsec] = data;

}

template <class T>
bool StampedData<T>::Get(const ros::Time& tStamp,T& data, const bool interpolate){
  const sec_t tSec   = tStamp.sec;
  const nsec_t tNsec = tStamp.nsec;
  if(bucket_.find(tSec) == bucket_.end()){
    return false; // do not exrapolate
  }

}

template <class T>
typename StampedData<T>::Iterator StampedData<T>::begin(){

  return bucket_.empty() ? nullptr : StampedData<T>::Iterator(bucket_.begin()->begin());

}

template <class T>
typename StampedData<T>::Iterator StampedData<T>::end(){

 return  bucket_.empty() ? nullptr : StampedData<T>::Iterator( std::prev(std::prev(bucket_.end())->end()) ); // last element in 2 dimesional contailer

}
*/


}

