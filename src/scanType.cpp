#include "dmapping/scanType.h"

namespace dmapping {

Scan::Parameters Scan::par_;

Scan::Scan(RingCloud::Ptr cloudInput){
  cloud_raw_ = cloudInput;
  ros::Time t;
  pcl_conversions::fromPCL(cloud_raw_->header.stamp, t);
  stamp_ = t.toSec();
  //fullcloud_->points.resize(par_.N_SCAN*par_.Horizon_SCAN);
  //projectPointCloud();
  //cout << ", cloud: " << GetRelTime(stamp_);
}

Cloud::Ptr Scan::GetCloud(){
  Cloud::Ptr cld(new Cloud());
  for(auto&& p : cloud_raw_->points){
    PointType pnt_tmp;
    pnt_tmp.x = p.x;
    pnt_tmp.y = p.y;
    pnt_tmp.z = p.z;
    pnt_tmp.intensity = p.intensity;
    cld->push_back(pnt_tmp);
  }
  return cld;
}
RingCloud::Ptr Scan::GetRingCloud(){
  return cloud_raw_;
}

double Scan::GetStamp() const{
  return stamp_;
}
/*
void Scan::projectPointCloud()
{
  const int cloudSize = cloud_raw_->points.size();
  // range image projection
  for (int i = 0; i < cloudSize; ++i)
  {
    pcl::PointXYZI thisPoint;Cloud
    thisPoint.x = cloud_raw_->points[i].x;
    thisPoint.y = cloud_raw_->points[i].y;
    thisPoint.z = cloud_raw_->points[i].z;
    thisPoint.intensity = cloud_raw_->points[i].intensity;

    const float range = pointDistance(thisPoint);
    if (range < 1000 || range > 0.1)
      continue;

    const int rowIdn = cloud_raw_->points[i].ring;
    if (rowIdn < 0 || rowIdn >= par_.N_SCAN)
      continue;

    //if (rowIdn % 1 != 0)
    //        continue;

    int columnIdn = -1;


    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
    static float ang_res_x = 360.0/float(par_.Horizon_SCAN);
    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + par_.Horizon_SCAN/2;
    if (columnIdn >= par_.Horizon_SCAN)
      columnIdn -= par_.Horizon_SCAN;


    if (columnIdn < 0 || columnIdn >= par_.Horizon_SCAN)
      continue;

    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
      continue;

    //thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

    rangeMat.at<float>(rowIdn, columnIdn) = range;

    int index = columnIdn + rowIdn * par_.Horizon_SCAN;
    fullCloud_->points[index] = thisPoint;

  }
*/

}
