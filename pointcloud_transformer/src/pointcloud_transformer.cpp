/**
* This node is based on the PCL tutorials: http://pointclouds.org/documentation/tutorials/
* This node is intended to be used as a utility helper function
*/

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <velodyne_pointcloud/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <ros/ros.h>
#include <memory>

class PointCloudTransformNode
{
private:
  float theta;
  float psi;
  std::shared_ptr<ros::NodeHandle> nh;
  ros::Subscriber     points_node_sub_;
	ros::Publisher      transformed_points_pub_;
public:
  PointCloudTransformNode(float theta, float psi): theta(theta), psi(psi) { }

  void initialize(std::shared_ptr<ros::NodeHandle> nh)
  {
    this->nh = nh;
    points_node_sub_ = nh->subscribe("velodyne_points", 1,
      &PointCloudTransformNode::CloudCallbackVelodyne, this
    );

		transformed_points_pub_ =
      nh->advertise<sensor_msgs::PointCloud2>("points_raw", 2);

  }

  void CloudCallbackVelodyne(
    const pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ConstPtr &in_sensor_cloud)
	{
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    
    transform_2.translation() << 0.5, 0.0, 1.8;

    // The same rotation matrix as before; theta radians around Z axis
    transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    transform_2.rotate (Eigen::AngleAxisf (psi,   Eigen::Vector3f::UnitX()));

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (
      new pcl::PointCloud<pcl::PointXYZ> ()
    );
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud (
      new pcl::PointCloud<velodyne_pointcloud::PointXYZIR> ()
    );
    pcl::transformPointCloud<velodyne_pointcloud::PointXYZIR> (*in_sensor_cloud, *transformed_cloud, transform_2);

		transformed_points_pub_.publish(transformed_cloud);
	}
};



int main (int argc, char** argv)
{
  ros::init(argc, argv, "static_pcl_transformer");
  float theta = std::atof(argv[1]);
  float psi   = std::atof(argv[2]);
  ROS_INFO("Using theta (pitch) %3f", theta);
  ROS_INFO("Using psi  (roll) %3f", psi);
  std::shared_ptr<ros::NodeHandle> nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
  PointCloudTransformNode pnode(theta, psi);
  pnode.initialize(nh);
  ros::spin();
}
