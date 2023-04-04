#include "ros/ros.h"

#include "iiwa_tools/GetIK.h"
#include "std_msgs/MultiArrayDimension.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

geometry_msgs::Pose target_pose;
ros::Publisher pub;
sensor_msgs::JointState joint_state_from_ik;
// std::vector<geometry_msgs::Pose> target_poses;

ros::ServiceClient *ik_clientPtr;
iiwa_tools::GetIK ik_serv;
std::vector<double> current_joint_pos;
std::vector<double> updated_joint_pos;
int count_ = 0;


void states_(sensor_msgs::JointState msg_states){
    current_joint_pos = msg_states.position;
}
// void states_fake(sensor_msgs::JointState msg_fake_states){
//     updated_joint_pos = msg_fake_states.position;
// }
void ref_pose_cb(geometry_msgs::PoseStamped msg){
    geometry_msgs::PoseStamped ref_pose_ = msg;
    ref_pose_.header.frame_id = "world";
    ref_pose_.header.stamp = ros::Time::now();
    target_pose.position = msg.pose.position;
    target_pose.orientation = msg.pose.orientation;

    ik_serv.request.poses = {target_pose};
    ik_serv.request.seed_angles.layout.dim.resize(2);
    ik_serv.request.seed_angles.layout.dim[0].size = 1;
    ik_serv.request.seed_angles.layout.dim[1].size = 7; 
    // std::cout << count_ << std::endl; 
    if(count_ == 0){  
        for(int i =0; i < 7; i++){
            ik_serv.request.seed_angles.data.push_back(current_joint_pos[i]);//{0., 0., 0., 0., 0., 0., 0.};
            // ik_serv.request.seed_angles.data.push_back(0);//{0., 0., 0., 0., 0., 0., 0.};
            
        }   
        // std::cout << "seeded0" << "--" << current_joint_pos[3] <<std::endl;
    }
    else{
        for(int i =0; i < 7; i++){

            ik_serv.request.seed_angles.data.push_back(updated_joint_pos[i]);//{0., 0., 0., 0., 0., 0., 0.};
            // ik_serv.request.seed_angles.data.push_back(0);//{0., 0., 0., 0., 0., 0., 0.};
            
        } 
        // std::cout << "seeded" << "--" << updated_joint_pos[3] <<std::endl;

    }
    joint_state_from_ik.header.stamp = ros::Time::now();
    joint_state_from_ik.name.resize(7);
    joint_state_from_ik.position.resize(7);
    joint_state_from_ik.velocity.resize(7);
    joint_state_from_ik.effort.resize(7);    
    joint_state_from_ik.name = {"iiwa_joint_1","iiwa_joint_2","iiwa_joint_3","iiwa_joint_4","iiwa_joint_5","iiwa_joint_6","iiwa_joint_7"};

    if (ik_clientPtr->call(ik_serv)){
        std::vector<double> joint_values = ik_serv.response.joints.data;
        joint_state_from_ik.position = ik_serv.response.joints.data;
        updated_joint_pos = ik_serv.response.joints.data;
        // std::cout << "updated" << "--" << updated_joint_pos[3] <<std::endl;
        
         
        // for (int i = 0; i < joint_values.size(); i++)
        // {
        //      std::cout << "iiwa_joint_" << i << ":" << "" << joint_values[i] <<std::endl;
            
        // }
        // ROS_INFO_STREAM("Next Pose");
    }
    else{
        ROS_ERROR("Failed to call service iiwa_ik_server");
    } 
    pub.publish(joint_state_from_ik);
    count_++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_ik");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    // target_pose.position.x =       0.3750;
    // target_pose.position.y =       0.3300;
    // target_pose.position.z =       0.3000;
    // target_pose.orientation.x =    -0.7073;
    // target_pose.orientation.y =    0.7068;
    // target_pose.orientation.z =    0.0016;
    // target_pose.orientation.w =    0.0016;

    ROS_INFO("Into the Check IK Node");
    ros::ServiceClient ik_client = nh.serviceClient<iiwa_tools::GetIK>("/iiwa/iiwa_ik_server");
    ik_clientPtr = &ik_client;
    ik_client.waitForExistence();
    // ros::Subscriber sub_fake_states_ = nh.subscribe("/joint_states_from_ik", 1, states_fake);
    ros::Subscriber sub_states_ = nh.subscribe("/iiwa/joint_states", 1, states_);
    ros::Subscriber sub = nh.subscribe("/cartesian_trajectory_generator/ref_pose", 1, ref_pose_cb);
    // iiwa_tools::GetIK ik_serv;

    // ik_serv.request.poses = target_poses; //{target_pose};
    // ik_serv.request.seed_angles.layout.dim.resize(2);
    // ik_serv.request.seed_angles.layout.dim[0].size = 1;
    // ik_serv.request.seed_angles.layout.dim[1].size = 7;    
    // ik_serv.request.seed_angles.data = {0., 0., 0., 0., 0., 0., 0.};

    // if (ik_clientPtr->call(ik_serv)){
    //     std::vector<double> joint_values = ik_serv.response.joints.data;
         
    //     for (int i = 0; i < joint_values.size(); i++)
    //     {
    //         std::cout << "iiwa_joint_" << i << ":" << "" << joint_values[i] <<std::endl;
    //     }
    // }
    // else{
    //     ROS_ERROR("Failed to call service iiwa_ik_server");
    // }
    pub = nh.advertise<sensor_msgs::JointState>("/joint_states_from_ik",1);
       
    ros::waitForShutdown();
}
