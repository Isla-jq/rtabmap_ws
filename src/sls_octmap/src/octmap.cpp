#include <rclcpp/node.hpp>
#include <iostream>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <typeinfo>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <string>
#include <Eigen/Dense>
#include <chrono>

class PCLOctMap : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rtabmap_subscribe_;
    sensor_msgs::msg::PointCloud2::SharedPtr SubPointCloud;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    octomap::OcTree tree;
    rclcpp::TimerBase::SharedPtr timer_;
    
    public:
    PCLOctMap();
    void timer_callback();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msgs);
};




PCLOctMap::PCLOctMap() : Node("pcl_to_octmap"),tree(0.05)
{
    rtabmap_subscribe_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rtabmap/cloud_map",10,
        std::bind(&PCLOctMap::pointCloudCallback, this, std::placeholders::_1));

    octomap_publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(0.5),
      [this]() { this->timer_callback(); });
      
};

void PCLOctMap::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msgs) {
    SubPointCloud = std::move(msgs);
};
void PCLOctMap::timer_callback() {
    if(SubPointCloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*SubPointCloud,*cloud);
        // 转换为 OctoMap 点云
        octomap::Pointcloud octo_cloud;
        for (auto& pt : cloud->points) {
            octo_cloud.push_back(pt.x, pt.y, pt.z);
        }
        // 插入到 octomap 中
        tree.clear(); // 每次都重新生成地图
        octomap::point3d sensor_origin(0.0, 0.0, 0.0);  // 设置原点，必要
        tree.insertPointCloud(octo_cloud, sensor_origin);

        // 转换为 ROS 消息
        octomap_msgs::msg::Octomap map_msg;
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();
        if (!octomap_msgs::fullMapToMsg(tree, map_msg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert Octomap to message!");
            return;
        }
        octomap_publisher_->publish(map_msg);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PCLOctMap>());
    rclcpp::shutdown();
    return 0;
}
