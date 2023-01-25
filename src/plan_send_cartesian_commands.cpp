#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pub;

void ref_pose_cb(geometry_msgs::PoseStamped msg){

    geometry_msgs::PoseStamped ref_pose_ = msg;
    ref_pose_.header.frame_id ="world";
    ref_pose_.header.stamp = ros::Time::now();
    std::cout<< ref_pose_ << std::endl;
    pub.publish(ref_pose_);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "plan_send_cartesian_commands");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Subscriber sub = nh.subscribe("/cartesian_trajectory_generator/ref_pose", 100, ref_pose_cb);
    pub  = nh.advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose",100);
    // ros::Duration(1).sleep();
    ros::waitForShutdown();


}