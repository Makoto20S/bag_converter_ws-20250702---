#include <ros/ros.h>
#include <ros/package.h>
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
#include <yaml-cpp/yaml.h>

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
    
    // 函数声明
    void loadExtrinsics();
    
public:
    BagToCloudInfoConverter(const std::string& device_id) : nh_("~"), device_id_(device_id)
    {
        // 初始化全局点云
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        // 获取参数
        nh_.param<std::string>("cloud_topic_" + device_id, cloud_topic_, "/location/ky_cloud_" + device_id);
        nh_.param<std::string>("pose_topic_" + device_id, pose_topic_, "/location/ky_pose_ins_" + device_id);
        nh_.param<std::string>("output_topic_" + device_id, output_topic_, "/cloud_info_" + device_id);
        nh_.param<std::string>("global_map_topic_" + device_id, global_map_topic_, "/global_map_" + device_id);
        std::string transformed_cloud_topic;
        nh_.param<std::string>("transformed_cloud_topic_" + device_id, transformed_cloud_topic, "/transformed_cloud_" + device_id);
        nh_.param<double>("voxel_size", voxel_size_, 0.1);
        nh_.param<int>("max_global_points", max_global_points_, 1000000);
        
        frame_id_ = "robot_" + device_id + "_base_link";
        
        // 配置体素滤波器
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        
        // 初始化发布者
        cloud_info_pub_ = nh_.advertise<bag_converter::cloud_info>(output_topic_, 10);
        global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic_, 1);
        transformed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(transformed_cloud_topic, 10);
        
        // 初始化订阅者
        cloud_sub_.subscribe(nh_, cloud_topic_, 10);
        pose_sub_.subscribe(nh_, pose_topic_, 10);
        
        // 初始化同步器
        sync_.reset(new Synchronizer(SyncPolicy(10), cloud_sub_, pose_sub_));
        sync_->registerCallback(boost::bind(&BagToCloudInfoConverter::syncCallback, this, _1, _2));
        
        ROS_INFO("Device %s converter initialized", device_id_.c_str());
        ROS_INFO("  Subscribing to cloud topic: %s", cloud_topic_.c_str());
        ROS_INFO("  Subscribing to pose topic: %s", pose_topic_.c_str());
        ROS_INFO("  Publishing cloud_info to: %s", output_topic_.c_str());
        ROS_INFO("  Publishing transformed cloud to: %s", transformed_cloud_topic.c_str()); 
        ROS_INFO("  Publishing global map to: %s", global_map_topic_.c_str());
        ROS_INFO("  Frame ID: %s", frame_id_.c_str());
        ROS_INFO("  Voxel size: %.3f", voxel_size_);
        
        // 加载外参
        loadExtrinsics();
    } 
    
    void syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                     const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        std::cout<<"Cloud time:"<< cloud_msg->header.stamp.sec<<"."<< cloud_msg->header.stamp.nsec <<std::endl;
        std::cout<<"Pose time:"<< pose_msg->header.stamp.sec<<"."<< pose_msg->header.stamp.nsec <<std::endl;
        // 检查时间戳是否完全一致
        if (cloud_msg->header.stamp != pose_msg->header.stamp) {
            ROS_WARN_THROTTLE(1.0, "Cloud and pose timestamps do not match exactly! Cloud: %d.%09d, Pose: %d.%09d",
                            cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nsec,
                            pose_msg->header.stamp.sec, pose_msg->header.stamp.nsec);
            return;  // 时间不一致，直接返回
        }
        
        // 创建cloud_info消息
        bag_converter::cloud_info cloud_info_msg;
        
        // 设置header
        cloud_info_msg.header = cloud_msg->header;
        
        // 从位姿消息中提取位置和姿态
        double x = pose_msg->pose.position.x;
        double y = pose_msg->pose.position.y;
        double z = pose_msg->pose.position.z;
        
        // 将四元数转换为欧拉角
        tf2::Quaternion quat(
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z,
            pose_msg->pose.orientation.w
        );
        
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        
        // 设置位姿信息
        cloud_info_msg.initialGuessX = x;
        cloud_info_msg.initialGuessY = y;
        cloud_info_msg.initialGuessZ = z;
        cloud_info_msg.initialGuessRoll = roll;
        cloud_info_msg.initialGuessPitch = pitch;
        cloud_info_msg.initialGuessYaw = yaw;
        
        // 设置可用性标志
        cloud_info_msg.imuAvailable = false;  // 没有IMU数据
        cloud_info_msg.odomAvailable = true;  // 有里程计数据
        
        // 设置点云数据
        cloud_info_msg.cloud_deskewed = *cloud_msg;
        cloud_info_msg.cloud_corner = *cloud_msg;    // 简化处理，实际应用中需要特征提取
        cloud_info_msg.cloud_surface = *cloud_msg;   // 简化处理，实际应用中需要特征提取
        
        // 设置其他字段的默认值
        cloud_info_msg.startRingIndex.clear();
        cloud_info_msg.endRingIndex.clear();
        cloud_info_msg.pointColInd.clear();
        cloud_info_msg.pointRange.clear();
        
        cloud_info_msg.imuRollInit = 0.0;
        cloud_info_msg.imuPitchInit = 0.0;
        cloud_info_msg.imuYawInit = 0.0;
        
        // 发布cloud_info消息
        cloud_info_pub_.publish(cloud_info_msg);
        
        // 发布TF变换
        publishTF(pose_msg);
        
        // 更新全局点云
        updateGlobalMap(cloud_msg, pose_msg);
        
        ROS_INFO_THROTTLE(2.0, "Device %s: Published cloud_info with pose [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Global map size: %lu", 
                         device_id_.c_str(), x, y, z, roll, pitch, yaw, global_map_->points.size());
    }
    
    void publishTF(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        geometry_msgs::TransformStamped transformStamped;
        
        transformStamped.header.stamp = pose_msg->header.stamp;
        transformStamped.header.frame_id = "map";
        // transformStamped.child_frame_id = "ouster/" + device_id_ + "/base_link";  // 改为雷达坐标系
        transformStamped.child_frame_id = "body";
        
        // 创建位姿变换矩阵
        Eigen::Affine3f pose_transform = Eigen::Affine3f::Identity();
        pose_transform.translation() << pose_msg->pose.position.x,
                                       pose_msg->pose.position.y,
                                       pose_msg->pose.position.z;
        
        Eigen::Quaternionf quat(
            pose_msg->pose.orientation.w,
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z
        );
        quat.normalize();
        pose_transform.linear() = quat.toRotationMatrix();
        
        // 结合外参变换：从雷达系到全局系的完整变换
        Eigen::Affine3f combined_transform = pose_transform * lidar_to_imu_transform_;
        
        // 提取变换结果
        Eigen::Vector3f translation = combined_transform.translation();
        Eigen::Quaternionf rotation(combined_transform.linear());
        rotation.normalize();
        
        // 设置TF变换
        transformStamped.transform.translation.x = translation.x();
        transformStamped.transform.translation.y = translation.y();
        transformStamped.transform.translation.z = translation.z();
        
        transformStamped.transform.rotation.x = rotation.x();
        transformStamped.transform.rotation.y = rotation.y();
        transformStamped.transform.rotation.z = rotation.z();
        transformStamped.transform.rotation.w = rotation.w();
        
        tf_broadcaster_.sendTransform(transformStamped);
    }
    
    void updateGlobalMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                        const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        // 转换ROS点云到PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *current_cloud);
        
        // // 创建变换矩阵
        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        
        // // 设置平移
        // transform.translation() << pose_msg->pose.position.x,
        //                            pose_msg->pose.position.y,
        //                            pose_msg->pose.position.z;
        
        // // 设置旋转
        // tf2::Quaternion quat(
        //     pose_msg->pose.orientation.x,
        //     pose_msg->pose.orientation.y,
        //     pose_msg->pose.orientation.z,
        //     pose_msg->pose.orientation.w
        // );
        
        // tf2::Matrix3x3 rotation_matrix(quat);
        // Eigen::Matrix3f eigen_rotation;
        // for (int i = 0; i < 3; i++) {
        //     for (int j = 0; j < 3; j++) {
        //         eigen_rotation(i, j) = rotation_matrix[i][j];
        //     }
        // }
        // transform.linear() = eigen_rotation;


        // 创建变换矩阵
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // 设置平移
        transform.translation() << pose_msg->pose.position.x,
                                pose_msg->pose.position.y,
                                pose_msg->pose.position.z;

        // 设置旋转
        Eigen::Quaternionf quat(
            pose_msg->pose.orientation.w,
            pose_msg->pose.orientation.x,
            pose_msg->pose.orientation.y,
            pose_msg->pose.orientation.z
        );
        quat.normalize(); 
        transform.linear() = quat.toRotationMatrix();

        
        // 变换当前点云到全局坐标系
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        // 先应用雷达到IMU的外参变换，再应用位姿变换
        Eigen::Affine3f combined_transform = transform * lidar_to_imu_transform_;
        pcl::transformPointCloud(*current_cloud, *transformed_cloud, combined_transform);
        publishTransformedCloud(transformed_cloud, cloud_msg->header.stamp);
        
        // 将变换后的点云添加到全局地图
        *global_map_ += *transformed_cloud;
        
        // 如果点云过大，进行下采样
        if (global_map_->points.size() > max_global_points_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            voxel_filter_.setInputCloud(global_map_);
            voxel_filter_.filter(*filtered_cloud);
            global_map_ = filtered_cloud;
            
            ROS_INFO_THROTTLE(5.0, "Device %s: Global map downsampled from %lu to %lu points", 
                             device_id_.c_str(), global_map_->points.size() + transformed_cloud->points.size(), 
                             global_map_->points.size());
        }
        
        // 发布全局地图
        publishGlobalMap(cloud_msg->header.stamp);
    }
    
    void publishTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_cloud, 
        const ros::Time& stamp)
    {
        if (transformed_cloud->points.empty()) {
        return;
        }

        sensor_msgs::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*transformed_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = stamp;
        transformed_cloud_msg.header.frame_id = "map";  // 全局坐标系

    transformed_cloud_pub_.publish(transformed_cloud_msg);
    }

    void publishGlobalMap(const ros::Time& stamp)
    {
        if (global_map_->points.empty()) {
            return;
        }
        
        sensor_msgs::PointCloud2 global_map_msg;
        pcl::toROSMsg(*global_map_, global_map_msg);
        global_map_msg.header.stamp = stamp;
        global_map_msg.header.frame_id = "map";
        
        global_map_pub_.publish(global_map_msg);
    }
};

    void BagToCloudInfoConverter::loadExtrinsics()
    {
        try {
            std::string config_path = ros::package::getPath("bag_converter") + "/config/extrinsics.yaml";
            YAML::Node config = YAML::LoadFile(config_path);
            
            std::string extrinsic_key = "extrinsic_" + device_id_;
            if (config[extrinsic_key]) {
                double x = config[extrinsic_key]["x"].as<double>();
                double y = config[extrinsic_key]["y"].as<double>();
                double z = config[extrinsic_key]["z"].as<double>();
                double roll = config[extrinsic_key]["roll"].as<double>();
                double pitch = config[extrinsic_key]["pitch"].as<double>();
                double yaw = config[extrinsic_key]["yaw"].as<double>();
                
                // 创建雷达到IMU的变换矩阵
                lidar_to_imu_transform_ = Eigen::Affine3f::Identity();
                lidar_to_imu_transform_.translation() << x, y, z;
                
                Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
                Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
                Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
                lidar_to_imu_transform_.linear() = (yawAngle * pitchAngle * rollAngle).matrix();
                
                ROS_INFO("Device %s: Loaded extrinsics [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                        device_id_.c_str(), x, y, z, roll, pitch, yaw);
            } else {
                ROS_ERROR("Device %s: Extrinsic parameters not found in config file", device_id_.c_str());
                lidar_to_imu_transform_ = Eigen::Affine3f::Identity();
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to load extrinsics: %s", e.what());
            lidar_to_imu_transform_ = Eigen::Affine3f::Identity();
        }
    }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_bag_to_cloud_info_converter");
    
    // 创建两个设备的转换器
    BagToCloudInfoConverter converter_0("0");
    BagToCloudInfoConverter converter_1("1");
    
    ROS_INFO("Starting dual-device real-time bag to cloud_info converter with global mapping...");
    ros::spin();
    
    return 0;
}
