#ifndef BAG_TO_CLOUD_INFO_CONVERTER_H
#define BAG_TO_CLOUD_INFO_CONVERTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "bag_converter/cloud_info.h"

class BagToCloudInfoConverter
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_info_pub_;
    ros::Publisher global_map_pub_;
    ros::Publisher transformed_cloud_pub_;
    
    // TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 使用message_filters进行时间同步
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
    
    std::string cloud_topic_;
    std::string pose_topic_;
    std::string output_topic_;
    std::string global_map_topic_;
    std::string device_id_;
    std::string frame_id_;
    
    // 全局点云存储
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    
    // 参数
    double voxel_size_;
    int max_global_points_;
    
    // 雷达到IMU的外参
    Eigen::Affine3f lidar_to_imu_transform_;
    
    // 位置跟踪变量
    bool first_pose_received_;
    geometry_msgs::Point last_published_position_;
    double distance_threshold_;
    
    // 私有方法声明
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    bool shouldPublish(const geometry_msgs::Point& current_position);
    void publishTF(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void updateGlobalMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                        const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void publishTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_cloud, 
                                const ros::Time& stamp);
    void publishGlobalMap(const ros::Time& stamp);
    
public:
    // 构造函数
    BagToCloudInfoConverter(const std::string& device_id);
    
    // 主要回调函数
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                     const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};

#endif // BAG_TO_CLOUD_INFO_CONVERTER_H