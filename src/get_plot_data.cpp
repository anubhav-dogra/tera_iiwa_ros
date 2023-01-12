#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"
#include "iiwa_tools/GetFK.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState joint_positions;
int n;

void iiwa_output_callback(iiwa_driver::AdditionalOutputs incoming_msg){
    std::cout << incoming_msg.commanded_torques << std::endl;
    //incoming_msg.external_torques;
    //incoming_msg.commanded_positions;

}
void iiwa_jointstates_callback(sensor_msgs::JointState inflow_j_states){ 
    int n = inflow_j_states.name.size();
    joint_positions.name.resize(n);
    joint_positions.position.resize(n);
    for (int i = 0; i < n; ++i)
    {
        joint_positions.position[i] = inflow_j_states.position[i];
        //std::cout << joint_positions.position[i] << std::endl;
    }
    
} 
int main(int argc, char **argv){
    ros::init(argc, argv, "get_plot_data");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("into the get_plot Node");
    
    ros::Subscriber sub_add = nh.subscribe("/additional_outputs", 10, iiwa_output_callback);
    ros::Subscriber sub_joint_states = nh.subscribe("/iiwa/joint_states", 10, iiwa_jointstates_callback);
    ros::ServiceClient client = nh.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
    iiwa_tools::GetFK srv_get_fk;


    ros::waitForShutdown();

}