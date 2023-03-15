#include "ros/ros.h"

#include <pluginlib/class_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>

std::vector<double> seed;
std::vector<double> ik_solution;
std::vector<geometry_msgs::Pose> pose;
ros::ServiceClient *ik_clientPtr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_ik");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    pluginlib::ClassLoader<kinematics::KinematicsBase> ikfast_loader("moveit_core","kinematics::KinematicsBase");
    try{
        boost::shared_ptr<kinematics::KinematicsBase>
        ikfast(ikfast_loader.createClassInstance("iiwa14_rs_tool_manipulator/IKFastKinematicsPlugin"));
        ikfast->initialize("robot_description","manipulator","iiwa_link_0", "tool_link_ee", 0.005);
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL("Unable to load plugin: %s", ex.what());
    }
    ros::waitForShutdown();
}
