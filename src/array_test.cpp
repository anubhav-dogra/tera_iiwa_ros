#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "array_test");
    ros::NodeHandle nh_array;
    ros::Publisher pub = nh_array.advertise<geometry_msgs::PoseStamped>("cartesian_trajectory_generator/new_goal",1);
    geometry_msgs::PoseArray::ConstPtr pose_array_dZ = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/tf_array_out_dZ");
    geometry_msgs::PoseArray::ConstPtr pose_array = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/tf_array_out");
    geometry_msgs::PoseStamped point_now_dZ;
    geometry_msgs::PoseStamped point_now;
    //  if (pose_array_dZ) {
    //     ROS_INFO("Received PoseArray with %zu poses", pose_array_dZ->poses.size());

    //     // You can access individual poses in the PoseArray
    //     for (const auto& pose : pose_array_dZ->poses) {
    //         ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", pose.position.x, pose.position.y, pose.position.z);
    //         ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    //     }
    // } else {
    //     ROS_ERROR("Timeout waiting for PoseArray message.");
    // }
    
    point_now.header.frame_id = "world";
    point_now_dZ.header.frame_id = "world";
    int array_size = pose_array->poses.size();
    std::cout << "array_size" << array_size << std::endl;
    for(int i = 0; i < array_size; i++) //array_size
      { 
        point_now_dZ.header.stamp = ros::Time::now();
        point_now_dZ.pose.position.x = pose_array_dZ->poses[i].position.x;
        point_now_dZ.pose.position.y = pose_array_dZ->poses[i].position.y;
        point_now_dZ.pose.position.z = pose_array_dZ->poses[i].position.z;
        point_now_dZ.pose.orientation.x = pose_array_dZ->poses[i].orientation.x;
        point_now_dZ.pose.orientation.y = pose_array_dZ->poses[i].orientation.y;
        point_now_dZ.pose.orientation.z = pose_array_dZ->poses[i].orientation.z;
        point_now_dZ.pose.orientation.w = pose_array_dZ->poses[i].orientation.w;

        // std::cout << "point_now_dZ" << point_now_dZ << std::endl;

        pub.publish(point_now_dZ);
        if(i == 0){ros::Duration(5).sleep();}
        else{ros::Duration(1).sleep();}
        
        point_now.header.stamp = ros::Time::now();
        point_now.pose.position.x = pose_array->poses[i].position.x;
        point_now.pose.position.y = pose_array->poses[i].position.y;
        point_now.pose.position.z = pose_array->poses[i].position.z;
        point_now.pose.orientation.x = pose_array->poses[i].orientation.x;
        point_now.pose.orientation.y = pose_array->poses[i].orientation.y;
        point_now.pose.orientation.z = pose_array->poses[i].orientation.z;
        point_now.pose.orientation.w = pose_array->poses[i].orientation.w;

        // std::cout << "point_now" << point_now << std::endl;

        pub.publish(point_now);
        ros::Duration(1).sleep();
      }



    return 0;
}


