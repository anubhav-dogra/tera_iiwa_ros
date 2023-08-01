#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// pcl includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub_poses;
ros::Subscriber sub_pcloud;
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
Eigen::Vector3d cross_prod(Eigen::Vector3d vector_1,Eigen::Vector3d vector_2);
sensor_msgs::PointCloud2 cloud_read;

Eigen::Vector3d cross_prod(Eigen::Vector3d vector_1,Eigen::Vector3d vector_2)
{
  Eigen::Vector3d out_vector;
  out_vector[0] = vector_1[1]*vector_2[2]-vector_1[2]*vector_2[1];
  out_vector[1] = vector_1[2]*vector_2[0]-vector_1[0]*vector_2[2];
  out_vector[2] = vector_1[0]*vector_2[1]-vector_1[1]*vector_2[0];
  return out_vector;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_input);  
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_input);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_filtered);
  int v_cloudsize2 = (cloud_filtered->width) * (cloud_filtered->height);
  //for (int i = 0; i < v_cloudsize2; i++) {
  // std::cout << cloud_input->points[0] << '\n';
  //}
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud (cloud_filtered);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  normal_estimation.setRadiusSearch (0.03);
  //normal_estimation.useSensorOriginAsViewPoint();
  normal_estimation.setViewPoint(0, 0, 10);
  normal_estimation.compute(*cloud_normals);

  //Convert to pose
  int cloudsize = (cloud_filtered->width) * (cloud_filtered->height);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  geometry_msgs::PoseArray output_pose_array;
  for (int i = 0; i < cloudsize; i++) {
    pose(0,3) = cloud_filtered->points[i].x;
    pose(1,3) = cloud_filtered->points[i].y;
    pose(2,3) = cloud_filtered->points[i].z;
    Eigen::Vector3d axis_normal(cloud_normals->points[i].normal_x, cloud_normals->points[i].normal_y, cloud_normals->points[i].normal_z);
    Eigen::Vector3d axis_x = cross_prod(axis_normal,Eigen::Vector3d::UnitZ());
    axis_x.normalize();
    Eigen::Vector3d axis_y = cross_prod(axis_normal,axis_x);
    axis_y.normalize();
    pose(0,0) = axis_x[0],pose(0,1) = axis_y[0],pose(0,2) = axis_normal[0];
    pose(1,0) = axis_x[1],pose(1,1) = axis_y[1],pose(1,2) = axis_normal[1];
    pose(2,0) = axis_x[2],pose(2,1) = axis_y[2],pose(2,2) = axis_normal[2];
    Eigen::Matrix3d rot;
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        rot(j,k) = pose(j,k);
      }
    }
    //std::cout<<axis_normal<<endl;
    //std::cout<<rot<<endl;
    Eigen::Quaterniond q;
    q = Eigen::Matrix3d(rot);
    q.normalize();
    
    // Minimize Pose difference from Calculated to the original
    tf::Quaternion q_tf;
    tf::quaternionEigenToTF(q,q_tf);
    // std::cout<<q_tf.getX() << "="<< q_tf.getY() << "="<< q_tf.getZ()<< "=" << q_tf.getW() << std::endl;
    tf::Matrix3x3 q_tf_m(q_tf);
    double roll, pitch, yaw;
    q_tf_m.getRPY(roll, pitch, yaw); 
    Eigen::Matrix3d m_ = m_; // rotation about Z 
    m_(0,0) =  cos(-yaw), m_(0,1) = -sin(-yaw),m_(0,2) = 0;
    m_(1,0) = sin(-yaw), m_(1,1) = cos(-yaw), m_(1,2) = 0;
    m_(2,0) = 0,m_(2,1) = 0,m_(2,2) = 1;
    Eigen::Matrix3d rot1 = rot*m_; // Apply Rotation
    Eigen::Quaterniond q_;
    q_ = Eigen::Matrix3d(rot1);
    q_.normalize();
    

    // To complete dZ distance from the target point
    geometry_msgs::Quaternion quat_msg;
    quat_msg.w = q_.w();quat_msg.x = q_.x();quat_msg.y = q_.y();quat_msg.z = q_.z();
    tf2::Quaternion q_tf_dz;
    tf2::convert(quat_msg,q_tf_dz);
    
    tf2::Vector3 translation_tp(pose(0,3),pose(1,3),pose(2,3));
    tf2::Transform transform_tp(q_tf_dz, translation_tp);

    geometry_msgs::Quaternion quat_msg_;
    quat_msg_.w = 1;quat_msg_.x = 0;quat_msg_.y = 0;quat_msg_.z = 0;
    tf2::Quaternion q_tf_dz_;
    tf2::convert(quat_msg_,q_tf_dz_);
    // std::cout << q_tf_dz_.w() << q_tf_dz_.x() << q_tf_dz_.y() << q_tf_dz_.z() << std::endl;
    tf2::Vector3 translation_tp_(0,0,-0.025);
    tf2::Transform transform_tp_(q_tf_dz_, translation_tp_);

    // Apply transformation
    tf2:: Transform transformed_tp_cam = transform_tp*transform_tp_;

    geometry_msgs::TransformStamped static_transform_tp_cam;
    static_transform_tp_cam.header.stamp = ros::Time::now();
    static_transform_tp_cam.header.frame_id = "world";
    static_transform_tp_cam.child_frame_id = "goal_frame";
    static_transform_tp_cam.transform = tf2::toMsg(transformed_tp_cam);

    // Publish Poses on ROS topics
    output_pose_array.header.stamp = ros::Time::now();
    output_pose_array.header.frame_id = "camera_color_optical_frame";
    geometry_msgs::Pose output_pose;
    output_pose.position.x = static_transform_tp_cam.transform.translation.x;
    output_pose.position.y = static_transform_tp_cam.transform.translation.y;
    output_pose.position.z = static_transform_tp_cam.transform.translation.z;
    output_pose.orientation.x = static_transform_tp_cam.transform.rotation.x;
    output_pose.orientation.y = static_transform_tp_cam.transform.rotation.y;
    output_pose.orientation.z = static_transform_tp_cam.transform.rotation.z;
    output_pose.orientation.w = static_transform_tp_cam.transform.rotation.w;
    // push in array
    output_pose_array.poses.push_back(output_pose);
    
    // std::cout<<output_pose_array<<std::endl;

    /*std:cout << i << '\n';
    std::cout << q.vec() << '\n';
    std::cout << q.w() << '\n';*/
    /*std::cout << q.x() << '\n';    std::cout << q.y() << '\n';    std::cout << q.z() << '\n';*/  

  }
  pub_poses.publish(output_pose_array);
  return;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ComputePoses_dZ");
  ros::NodeHandle nh;
  // create a ros subscriber from the input point cloud.
  sub_pcloud = nh.subscribe("cropbox/output", 100, cloud_cb);
  pub_poses = nh.advertise<geometry_msgs::PoseArray>("pose_output", 100);

  ros::spin ();
}
