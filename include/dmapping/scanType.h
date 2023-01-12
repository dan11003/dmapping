
#include "dmapping/utility.h"
namespace dmapping {

class Scan{

public:

  struct Parameters
  {
    int N_SCAN = 16, Horizon_SCAN = 1800;

    void GetParametersFromRos(ros::NodeHandle& nh){
      nh.param<int>("/N_SCAN", N_SCAN, 16);
      nh.param<int>("/Horizon_SCAN", Horizon_SCAN, 1800);
    }
  };

  Scan(RingCloud::Ptr cloudInput);

  Cloud::Ptr GetCloud();

  double GetStamp() const;

private:

//  void projectPointCloud();

  RingCloud::Ptr cloud_raw_;
  Cloud::Ptr fullcloud_;

  double stamp_;

  /* Feature extraction */
  static Parameters par_;
  cv::Mat rangeMat;

};

}
