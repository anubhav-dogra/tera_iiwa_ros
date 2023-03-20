#include "ros/ros.h"

#include "iiwa_tools/GetIK.h"
#include "std_msgs/MultiArrayDimension.h"

geometry_msgs::Pose target_pose;
//std::vector<geometry_msgs::Pose> target_poses = {target_pose};

ros::ServiceClient *ik_clientPtr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_ik");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    geometry_msgs::Pose target_pose;
    target_pose.position.x =       0.3750;
    target_pose.position.y =       0.3300;
    target_pose.position.z =       0.3000;
    target_pose.orientation.x =    -0.7073;
    target_pose.orientation.y =    0.7068;
    target_pose.orientation.z =    0.0016;
    target_pose.orientation.w =    0.0016;

    ROS_INFO("Into the Check IK Node");
    ros::ServiceClient ik_client = nh.serviceClient<iiwa_tools::GetIK>("/iiwa/iiwa_ik_server");
    ik_clientPtr = &ik_client;
    ik_client.waitForExistence();
    iiwa_tools::GetIK ik_serv;

    ik_serv.request.poses = {target_pose};
    ik_serv.request.seed_angles.layout.dim.resize(2);
    ik_serv.request.seed_angles.layout.dim[0].size = 1;
    ik_serv.request.seed_angles.layout.dim[1].size = 7;    
    ik_serv.request.seed_angles.data = {0., 0., 0., 0., 0., 0., 0.};

    if (ik_clientPtr->call(ik_serv)){
        std::vector<double> joint_values = ik_serv.response.joints.data;
         
        for (int i = 0; i < joint_values.size(); i++)
        {
            std::cout << "iiwa_joint_" << i << ":" << "" << joint_values[i] <<std::endl;
        }
    }
    else{
        ROS_ERROR("Failed to call service iiwa_ik_server");
    }
       
    ros::waitForShutdown();
}
