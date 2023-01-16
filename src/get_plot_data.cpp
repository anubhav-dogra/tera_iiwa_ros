#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"
#include <iiwa_tools/GetFK.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

double joint_positions[6];
int n;
ros::ServiceClient *fk_clientPtr;
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
    //ROS_INFO("Subscribing to the joint_states");
    //std::cout <<  joint_positions << std::endl;
    //std::cout <<  positions_joints << std::endl;

    int n = inflow_j_states.name.size();
    //joint_positions.name.resize(n);
    //joint_positions.position.resize(n);
    for (int i = 0; i < n; ++i)
    {
        joint_positions[i] = inflow_j_states.position[i];
         //std::cout << joint_positions[i] << std::endl;
    
    } 
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
    }
    //std::cout <<  positions_joints << std::endl;

    iiwa_tools::GetFK get_fk_srv;
    //ROS_INFO("calling the service");
    get_fk_srv.request.joints = positions_joints;
    if (fk_clientPtr->call(get_fk_srv))
    {
       ROS_INFO("The iiwa_ee_state X: %f Y: %f, Z: %f", get_fk_srv.response.poses[0].position.x, get_fk_srv.response.poses[0].position.y, get_fk_srv.response.poses[0].position.z);
       ROS_INFO("The iiwa_ee_state X.O: %f Y.O: %f, Z.O: %f, W.O: %f", get_fk_srv.response.poses[0].orientation.x, get_fk_srv.response.poses[0].orientation.y, get_fk_srv.response.poses[0].orientation.z, get_fk_srv.response.poses[0].orientation.w);
    }
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "get_plot_data");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("into the get_plot Node");
    ros::ServiceClient fk_client = nh.serviceClient<iiwa_tools::GetFK>("/iiwa/iiwa_fk_server");
    fk_clientPtr = &fk_client;
    fk_client.waitForExistence();

    ros::Subscriber sub_add = nh.subscribe("/additional_outputs", 100, iiwa_output_callback);
    ros::Subscriber sub_joint_states = nh.subscribe("/iiwa/joint_states", 100, iiwa_jointstates_callback);



    ros::waitForShutdown();

}