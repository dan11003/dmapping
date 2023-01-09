#include "dmapping/utility.h"

namespace dmapping {

void PublishCloud(const std::string& topic, pcl::PointCloud<pointT>& cloud, const std::string& frame_id, const ros::Time& t){

  pcl_conversions::toPCL(t,cloud.header.stamp);
  cloud.header.frame_id = frame_id;

  static std::map<std::string, ros::Publisher> pubs;
  std::map<std::string, ros::Publisher>::iterator it = pubs.find(topic);
  static ros::NodeHandle nh("~");
  if (it == pubs.end()){
    pubs[topic] =  nh.advertise<pcl::PointCloud<pointT> >(topic,100);
  }
  pubs[topic].publish(cloud);
}
void PublishTF(const std::string& fixed_id, const std::string& frame_id, const Eigen::Affine3d& T, const ros::Time& t){
  static tf::TransformBroadcaster Tbr;
  tf::Transform Tf;
  std::vector<tf::StampedTransform> trans_vek;
  tf::transformEigenToTF(T, Tf);
  trans_vek.push_back(tf::StampedTransform(Tf, t, fixed_id, frame_id));
  Tbr.sendTransform(trans_vek);
}
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
  Eigen::AngleAxisd rollAngle(roll*M_PI/180.0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch*M_PI/180.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw*M_PI/180.0, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
  return q;
}
double GetRelTime(const double t){
  static double tinit = t;
  return t - tinit;
}

/*
ParamServer::ParamServer()
{
    cout << "pars "<< endl;
    nh.param<std::string>("/robot_id", robot_id, "roboat");

    nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "/velodyne_points");
    nh.param<std::string>("lio_sam/imuTopic", imuTopic, "/imu/data");

    nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

    nh.param<bool>("lio_sam/savePCD", savePCD, false);
    nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

    std::string sensorStr;
    nh.param<std::string>("lio_sam/sensor", sensorStr, "velodyne");
    if (sensorStr == "velodyne"){
        sensor = SensorType::VELODYNE;
    }
    else{
        ROS_ERROR_STREAM("Invalid sensor type (must be either 'velodyne' or 'ouster' or 'mulran'): " << sensorStr);
        ros::shutdown();
    }

    nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
    nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
    nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
    nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
    nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
    nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
    nh.param<float>("lio_sam/imuGravity", imuGravity, 9.81511);
    nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
    nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
    nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);
     cout << "pars "<< endl;

    usleep(100);
}

sensor_msgs::Imu ParamServer::imuConverter(const sensor_msgs::Imu& imu_in)
{
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}

std::string padZeros(int val, int num_digits) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

float pointDistance(PointType p)
{
    return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter)
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");

    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

*/
}
