#include "bag_to_cloud_info.h"
#include <yaml-cpp/yaml.h>

// 构造函数实现
BagToCloudInfoConverter::BagToCloudInfoConverter(const std::string& device_id) 
    : nh_(), device_id_(device_id)
{
    // 初始化全局点云
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 初始化位置跟踪变量
    first_pose_received_ = false;
    distance_threshold_ = 2.0;  // 2米阈值
    last_published_position_.x = 0.0;
    last_published_position_.y = 0.0;
    last_published_position_.z = 0.0;
    
    // 获取参数
    nh_.param<std::string>("cloud_topic_" + device_id, cloud_topic_, "/location/ky_cloud_" + device_id);
    nh_.param<std::string>("pose_topic_" + device_id, pose_topic_, "/location/ky_pose_ins_" + device_id);
    nh_.param<std::string>("output_topic_" + device_id, output_topic_, "/cloud_info_" + device_id);
    nh_.param<std::string>("global_map_topic_" + device_id, global_map_topic_, "/global_map_" + device_id);
    std::string transformed_cloud_topic;
    nh_.param<std::string>("transformed_cloud_topic_" + device_id, transformed_cloud_topic, "/transformed_cloud_" + device_id);
    nh_.param<double>("voxel_size", voxel_size_, 0.1);
    nh_.param<int>("max_global_points", max_global_points_, 1000000);
    nh_.param<double>("distance_threshold", distance_threshold_, 2.0);  // 可配置的距离阈值
    
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
    std::string extrinsic_key = "extrinsic_" + device_id_;

    double x, y, z, roll, pitch, yaw;
    if (nh_.getParam(extrinsic_key + "/x", x) &&
        nh_.getParam(extrinsic_key + "/y", y) &&
        nh_.getParam(extrinsic_key + "/z", z) &&
        nh_.getParam(extrinsic_key + "/roll", roll) &&
        nh_.getParam(extrinsic_key + "/pitch", pitch) &&
        nh_.getParam(extrinsic_key + "/yaw", yaw)) {
        
        // 创建雷达到IMU的变换矩阵
        lidar_to_imu_transform_ = Eigen::Affine3f::Identity();
        lidar_to_imu_transform_.translation() << x, y, z;
        
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        lidar_to_imu_transform_.linear() = (yawAngle * pitchAngle * rollAngle).matrix();
        
        ROS_INFO("Device %s: Loaded extrinsics from parameter server [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                device_id_.c_str(), x, y, z, roll, pitch, yaw);
        } else {
            ROS_ERROR("Device %s: Failed to load extrinsic parameters from parameter server", device_id_.c_str());
            lidar_to_imu_transform_ = Eigen::Affine3f::Identity();
        }
}

// 计算两点之间的距离
double BagToCloudInfoConverter::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// 检查是否需要发布（距离阈值判断）
bool BagToCloudInfoConverter::shouldPublish(const geometry_msgs::Point& current_position)
{
    if (!first_pose_received_) {
        first_pose_received_ = true;
        last_published_position_ = current_position;
        return true;  // 第一次接收到位姿时发布
    }
    
    double distance = calculateDistance(current_position, last_published_position_);
    if (distance >= distance_threshold_) {
        last_published_position_ = current_position;
        return true;
    }
    
    return false;
}

// 主要同步回调函数
void BagToCloudInfoConverter::syncCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
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
    
    // 检查是否需要发布（基于距离阈值）
    geometry_msgs::Point current_position = pose_msg->pose.position;
    bool should_publish = shouldPublish(current_position);
    
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
    
    // 创建位姿变换矩阵
    Eigen::Affine3f pose_transform = Eigen::Affine3f::Identity();
    pose_transform.translation() << x, y, z;
    
    Eigen::Quaternionf eigen_quat(
        pose_msg->pose.orientation.w,
        pose_msg->pose.orientation.x,
        pose_msg->pose.orientation.y,
        pose_msg->pose.orientation.z
    );
    eigen_quat.normalize();
    pose_transform.linear() = eigen_quat.toRotationMatrix();
    
    // 结合外参变换：从雷达系到全局系的完整变换
    Eigen::Affine3f combined_transform = pose_transform * lidar_to_imu_transform_;
    
    // 提取变换后的位置
    Eigen::Vector3f transformed_translation = combined_transform.translation();
    
    // 提取变换后的旋转并转换为欧拉角
    Eigen::Quaternionf transformed_rotation(combined_transform.linear());
    transformed_rotation.normalize();
    
    // 转换为tf2四元数以便提取欧拉角
    tf2::Quaternion transformed_quat(
        transformed_rotation.x(),
        transformed_rotation.y(),
        transformed_rotation.z(),
        transformed_rotation.w()
    );
    
    tf2::Matrix3x3 transformed_mat(transformed_quat);
    double transformed_roll, transformed_pitch, transformed_yaw;
    transformed_mat.getRPY(transformed_roll, transformed_pitch, transformed_yaw);
    
    // 设置变换后的位姿信息
    cloud_info_msg.initialGuessX = transformed_translation.x();
    cloud_info_msg.initialGuessY = transformed_translation.y();
    cloud_info_msg.initialGuessZ = transformed_translation.z();
    cloud_info_msg.initialGuessRoll = transformed_roll;
    cloud_info_msg.initialGuessPitch = transformed_pitch;
    cloud_info_msg.initialGuessYaw = transformed_yaw;
    
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
    
    // 只有当距离变化超过阈值时才发布TF变换和更新全局地图
    if (should_publish) {
        // 发布TF变换
        publishTF(pose_msg);
        
        // 更新全局点云
        updateGlobalMap(cloud_msg, pose_msg);
        
        ROS_INFO("Device %s: Published TF and global map at position [%.2f, %.2f, %.2f], distance moved: %.2f m", 
                 device_id_.c_str(), x, y, z, 
                 first_pose_received_ ? calculateDistance(current_position, last_published_position_) : 0.0);
    }
    
    ROS_INFO_THROTTLE(2.0, "Device %s: Published cloud_info with pose [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], Global map size: %lu", 
                     device_id_.c_str(), x, y, z, roll, pitch, yaw, global_map_->points.size());
}

// 发布TF变换
void BagToCloudInfoConverter::publishTF(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = pose_msg->header.stamp;
    // 根据设备ID设置不同的frame_id
    if (device_id_ == "0") {
        transformStamped.header.frame_id = "map";
    } else if (device_id_ == "1") {
        transformStamped.header.frame_id = "1/odom";
    } else {
        transformStamped.header.frame_id = "map";  // 默认值
    }
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

// 更新全局地图
void BagToCloudInfoConverter::updateGlobalMap(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // 转换ROS点云到PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *current_cloud);
    
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

// 发布变换后的点云
void BagToCloudInfoConverter::publishTransformedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformed_cloud, 
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

// 发布全局地图
void BagToCloudInfoConverter::publishGlobalMap(const ros::Time& stamp)
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