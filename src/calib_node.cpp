#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!
#include "dmapping/utility.h"
//#include "lio_sam/cloud_info.h"
#include <pcl/PCLPointCloud2.h>
#include "dmapping/stampeddata.h"
using namespace dmapping;

// Use the Velodyne point format as a common representation
struct VelodynePointXYZIRT // see lio-sam for more options
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring) (float, time, time)
                                   )
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;


    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;

    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;
    dmapping::ImuHandler imuBuffer;


public:
    ImageProjection():
    deskewFlag(0)
    {
        cout << "IP" <<  imuTopic << endl;
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        cout << "starting ImageProjection::imuHandler " <<  imuTopic << endl;
        cout << "starting ImageProjection::cloudHandler " <<  pointCloudTopic<< endl;

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);



        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        sensor_msgs::Imu thisImu = *imuMsg;
        thisImu.header.stamp.nsec = thisImu.header.stamp.nsec*1000;
        static sensor_msgs::Imu prevImu = thisImu;
        if(  thisImu.header.stamp - prevImu.header.stamp < ros::Duration(0.0005) )
          return;
        else
          prevImu = thisImu;
        imuBuffer.Add(thisImu);
        /*
        static tf::TransformBroadcaster Tbr;
        const double tsearch = thisImu.header.stamp.toSec() - 1.0;
        Eigen::Quaterniond qPrev;
        if(imuBuffer.Get(tsearch , qPrev)){
          Eigen::Affine3d tImuPose(qPrev);
          tf::Transform Tf;
          std::vector<tf::StampedTransform> trans_vek;
          tf::transformEigenToTF(tImuPose, Tf);
          trans_vek.push_back(tf::StampedTransform(Tf, imuMsg->header.stamp, "imu_fixed", "imu_past"));
          Tbr.sendTransform(trans_vek);
        }*/
    }

    bool CanBeDeskewed()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        // make sure IMU data available for the scan
        if (!imuBuffer.TimeContained(timeScanCur))
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }
        return true;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;
        if(!CanBeDeskewed()){
          cout << "problem" << endl;
          return;
        }
        else{
          cout << "can be deskewed, there is data" << endl;
          Deskew();
          Publish();
        }



    }
    void Deskew(){
      float tmax = -1, tmin = 1;

      for(auto&& p : laserCloudIn->points){
        tmin = std::min(tmin, p.time);
        tmax = std::max(tmax, p.time);
        //cout << p.ring << ", " << p.time << endl;
      }
      cout << "max: " << tmax << ", min: " << tmin << endl;



    }
    void Publish(){

    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();

        cloudQueue.pop_front();
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);


        // get timestamp
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }








    void cloudExtraction()
    {

    }
    
    void publishClouds()
    {

    }
};

int main(int argc, char** argv)
{
    cout << "start calib" << endl;
    ros::init(argc, argv, "calib");
    ImageProjection IP;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    cout << "End calib" << endl;
    
    return 0;
}
