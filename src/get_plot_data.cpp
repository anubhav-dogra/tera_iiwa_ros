#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"

void iiwa_output_callback(iiwa_driver::AdditionalOutputs incoming_msg){
    std::cout << incoming_msg.commanded_torques << std::endl;
    //incoming_msg.external_torques;
    //incoming_msg.commanded_positions;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "get_plot_data");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("into the get_plot Node");
    
    ros::Subscriber sub_add = nh.subscribe("/additional_outputs", 10, iiwa_output_callback);

    ros::waitForShutdown();

}