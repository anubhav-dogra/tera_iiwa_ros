#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pub_send;


void recieve_cb(geometry_msgs::PoseArray pose_array)
{
    
    geometry_msgs::PoseStamped single_pose_stamped;
    single_pose_stamped.header.stamp = ros::Time::now();
    single_pose_stamped.header.frame_id = "iiwa_link_0";
    //int array_size = pose_array.poses.size();
    //for(int i =0; i < array_size; i++){
        single_pose_stamped.pose = pose_array.poses[0];
        //std::cout<<single_pose_stamped<<std::endl;
        pub_send.publish(single_pose_stamped);
         ros::Duration(5.0).sleep();
    //}
    

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_new_goals");
    ros::NodeHandle nh_send;
    ros::Subscriber sub_array = nh_send.subscribe<geometry_msgs::PoseArray>("tf_array_out",1, recieve_cb);
    pub_send = nh_send.advertise<geometry_msgs::PoseStamped>("cartesian_trajectory_generator/new_goal",1);
    //  ros::Duration(20.0).sleep();
    ros::spin();
}
