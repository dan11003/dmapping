#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!
#include "dmapping/utility.h"
//#include "lio_sam/cloud_info.h"
#include <pcl/PCLPointCloud2.h>
#include "dmapping/dataHandler.h"
#include "dmapping/dmappingfuser.h"
using namespace dmapping;




int main(int argc, char** argv)
{
    cout << "start dmapping" << endl;
    ros::init(argc, argv, "dmapping");
    usleep(1000*1000);
    LidarBatch fuser;
    while(ros::ok){
      fuser.Visualize();
    }

    cout << "End dmapping" << endl;
    char c = getchar();
    
    return 0;
}
