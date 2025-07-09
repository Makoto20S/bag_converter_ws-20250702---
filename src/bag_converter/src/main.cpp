#include "bag_to_cloud_info.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_to_cloud_info_node");
    
    // 创建两个设备的转换器实例
    BagToCloudInfoConverter converter_0("0");
    BagToCloudInfoConverter converter_1("1");
    
    ROS_INFO("Bag to cloud info converter node started");
    
    ros::spin();
    
    return 0;
}
