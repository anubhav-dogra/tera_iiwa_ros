#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"
#include <iiwa_tools/GetFK.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

double joint_positions[6];
int n;
// Helper function to set value in a MultiArray
void set_multi_array(std_msgs::Float64MultiArray& array, size_t i, size_t j, double val)
{
    assert(array.layout.dim.size() == 2);
    size_t offset = array.layout.data_offset;

    array.data[offset + i * array.layout.dim[0].stride + j] = val;
}
void iiwa_output_callback(iiwa_driver::AdditionalOutputs incoming_msg){
    std::cout << incoming_msg.commanded_torques << std::endl;
    //incoming_msg.external_torques;
    //incoming_msg.commanded_positions;

}
void iiwa_jointstates_callback(sensor_msgs::JointState inflow_j_states){ 
    int n = inflow_j_states.name.size();
    //joint_positions.name.resize(n);
    //joint_positions.position.resize(n);
    for (int i = 0; i < n; ++i)
    {
        joint_positions[i] = inflow_j_states.position[i];
         //std::cout << joint_positions[i] << std::endl;
    
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
    ros::ServiceClient fk_client = nh.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
    fk_client.waitForExistence();
    std_msgs::Float64MultiArray positions_joints;
    positions_joints.layout.dim.resize(2);
    positions_joints.layout.data_offset = 0;
    positions_joints.layout.dim[0].size = 1;
    positions_joints.layout.dim[0].stride = 7;
    positions_joints.layout.dim[1].size = 7;
    positions_joints.layout.dim[1].stride = 0;
    positions_joints.data.resize(1*7);
    for (int i = 0; i < n; ++i)
    {
        positions_joints.data[i] = joint_positions[i];
        std::cout <<  joint_positions[i] << std::endl;

    }
    //std::cout <<  joint_positions << std::endl;
    //std::cout <<  positions_joints << std::endl;


    ros::waitForShutdown();

}