#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/WrenchStamped.h"
#include <cstdlib>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "tera_iiwa_ros/ForceZConfig.h"

float desired_force = 3.0;
class ForceController{
    private:
        double curr_force=0.0, error_fz=0.0, dFe=0.0, dZ=0.0;
        double curr_force_z=0.0, curr_force_x=0.0, curr_force_y=0.0, curr_torque_x=0.0, curr_torque_y=0.0, curr_torque_z=0.0;
        double prev_error_roll=0.0, prev_error_pitch=0.0, error_d_roll=0.0, error_d_pitch=0.0;
        ros::Publisher pose_pub;
        ros::Publisher pub_check;
        ros::Subscriber wrench_sub;
        tf2::Transform transform_base_ee;
        geometry_msgs::TransformStamped transformStamped_base_to_end;
        geometry_msgs::TransformStamped transformStamped_goal;
        double Kf = 10000.0;
        double Kp = 1.0/Kf;
        double Kd = 0.01/Kf;
        
        // double dt = 0.1;
        double prev_error_fz = 0.0;
        bool first_run = true;
        double alpha = 0.9;
        double smoothed_dFe = 0.0;
        double smoothed_drolle=0.0;
        double smoothed_dpitche=0.0;
        double Kp_orientation_x = 1.0/Kf;
        double Kp_orientation_y = 1.0/Kf;
        double Kd_orientation = 0.01/Kf;
        double set_value = 0.0;    
        tf2::Quaternion previous_quat;
        tf2::Quaternion new_quat;
        ros::Time current_time, previous_time;
        ros::Rate running_rate = 100;
        
    public:
    ForceController(ros::NodeHandle *nh){
      
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        //looking up transformation from base to end-effector of the robot
        try {
                transformStamped_base_to_end = tfBuffer.lookupTransform("iiwa_link_0", "tool_link_ee", ros::Time(0), ros::Duration(2));
            }
        catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());    
        }
        
        std::cout << transformStamped_base_to_end << std::endl;
        tf2::convert(transformStamped_base_to_end.transform.rotation, previous_quat);
        previous_quat.normalize();
        pose_pub = nh->advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose",1);
        pub_check = nh->advertise<geometry_msgs::PoseStamped>("pose_check",1);
        wrench_sub = nh->subscribe("/cartesian_wrench_tool_biased",10, &ForceController::callback_controller, this);
}
    void callback_controller(const geometry_msgs::WrenchStamped::ConstPtr& force_msg){
        
        curr_force_z = force_msg->wrench.force.z;
        curr_force_x = force_msg->wrench.force.x;
        curr_force_y = force_msg->wrench.force.y;
        curr_torque_x = force_msg->wrench.torque.x;
        curr_torque_y = force_msg->wrench.torque.y;
        curr_torque_z = force_msg->wrench.torque.z;

        error_fz = desired_force - abs(curr_force_z);
        double roll_error = -curr_force_x;
        double pitch_error = curr_force_y;
        // double roll_error = -curr_torque_x;
        // double pitch_error = curr_torque_y;

        if (first_run)
        {
            prev_error_fz = error_fz;
            prev_error_roll = roll_error;
            prev_error_pitch = pitch_error;
            previous_time = ros::Time::now();
            first_run = false;
        }
        current_time = ros::Time::now();
        double dt = (current_time - previous_time).toSec();
        previous_time = current_time;

        if (dt>0)
        {
            dFe = (error_fz - prev_error_fz)/dt;
            error_d_roll = (roll_error - prev_error_roll)/dt;
            error_d_pitch = (pitch_error - prev_error_pitch)/dt;
            prev_error_fz = error_fz;
            prev_error_roll = roll_error;
            prev_error_pitch = pitch_error;
            smoothed_dFe = alpha*smoothed_dFe + (1-alpha)*dFe;
            smoothed_drolle = alpha*smoothed_drolle + (1-alpha)*error_d_roll;
            smoothed_dpitche = alpha*smoothed_dpitche + (1-alpha)*error_d_pitch;

            dZ = Kp*error_fz + Kd*smoothed_dFe;
            double max_dZ = 0.00025;  // Maximum allowed movement per iteration
            if (fabs(dZ) > max_dZ) {
                dZ = copysign(max_dZ, dZ);  // Clamp the displacement
                // std::cout << "Clamping dZ to " << dZ << std::endl;
            }
            // double dRoll = Kp_orientation_x * roll_error + Kd_orientation * error_d_pitch + Kp_orientation_x *(0-curr_torque_x);
            // double dPitch = Kp_orientation_y * pitch_error + Kd_orientation * error_d_roll + Kp_orientation_y * (0-curr_torque_y);
            double dRoll = Kp_orientation_x*roll_error + Kd_orientation * smoothed_drolle; //+ Kp_orientation_x * curr_torque_x;
            double dPitch = Kp_orientation_y*pitch_error + Kd_orientation * smoothed_dpitche;// + Kp_orientation_y *-curr_torque_y;
            double max_dRoll = 0.001;  // Maximum allowed movement per iteration
            double max_dPitch = 0.001;  // Maximum allowed movement per iteration

            if (fabs(dRoll) > max_dRoll) {
                dRoll = copysign(max_dRoll, dRoll);  // Clamp the displacement
                std::cout << "Clamping dRoll "<< std::endl;
            }
            if (fabs(dPitch) > max_dPitch) {
                dPitch = copysign(max_dPitch, dPitch);  // Clamp the displacement
                std::cout << "Clamping dPitch "<< std::endl;
            }

            geometry_msgs::PoseStamped pose = update_pose(dZ, dRoll, dPitch);
            pose_pub.publish(pose);
        }
        //  running_rate.sleep();
    }

    void callback_dyn_param(tera_iiwa_ros::ForceZConfig &config, uint32_t level){
        desired_force = config.desired_force;
    }


    geometry_msgs::PoseStamped update_pose(double& dZ, double& dRoll, double& dPitch)
    {
        tf2::Quaternion quat_delta;
        quat_delta.setRPY(dRoll, dPitch, 0.0);
        quat_delta.normalize();
        new_quat = quat_delta * previous_quat;
        new_quat.normalize();

        transformStamped_goal.header.stamp = ros::Time::now();
        transformStamped_goal.header.frame_id = "tool_link_ee";
        transformStamped_goal.child_frame_id = "goal_point";
        // std::cout << "prev_z" << transformStamped_goal.transform.translation.z << std::endl;
        transformStamped_goal.transform.translation.x = 0.0;
        transformStamped_goal.transform.translation.y = 0.0;
        // if (abs(error_fz) < 0.01)
        // {
        //     std::cout << error_fz << std::endl;
        //     transformStamped_goal.transform.translation.z = set_value;
        //     std::cout <<"not changing z" << current_time <<std::endl;
        // }
        // else
        // {
        transformStamped_goal.transform.translation.z += dZ;
        // set_value = transformStamped_goal.transform.translation.z;
        // }
        
        // transformStamped_goal.transform.rotation.x = 0;
        // transformStamped_goal.transform.rotation.y = 0;
        // transformStamped_goal.transform.rotation.z = 0;
        // transformStamped_goal.transform.rotation.w = 1;
        transformStamped_goal.transform.rotation.x = quat_delta.getX();
        transformStamped_goal.transform.rotation.y = quat_delta.getY();
        transformStamped_goal.transform.rotation.z = quat_delta.getZ();
        transformStamped_goal.transform.rotation.w = quat_delta.getW();
        // std::cout << transformStamped_goal << std::endl;
        tf2::Vector3 translation_goal(transformStamped_goal.transform.translation.x,
                                        transformStamped_goal.transform.translation.y,
                                        transformStamped_goal.transform.translation.z);

        tf2::Quaternion quat_tf_goal;
        tf2::convert(transformStamped_goal.transform.rotation, quat_tf_goal);
        quat_tf_goal.normalize();
        tf2::Transform transform_goal(quat_tf_goal,translation_goal);

        tf2::Vector3 translation_base_ee(transformStamped_base_to_end.transform.translation.x,transformStamped_base_to_end.transform.translation.y,transformStamped_base_to_end.transform.translation.z);
        tf2::Quaternion quat_tf_base_end;
        tf2::convert(transformStamped_base_to_end.transform.rotation, quat_tf_base_end);
        quat_tf_base_end.normalize();
        tf2::Transform transform_base_ee(quat_tf_base_end,translation_base_ee);

        tf2:: Transform transformed_goal_base = transform_base_ee*transform_goal;

        geometry_msgs::TransformStamped static_transform_goal_base;
        static_transform_goal_base.header.stamp = ros::Time::now();
        static_transform_goal_base.header.frame_id = "world";
        static_transform_goal_base.child_frame_id = "goal_frame";
        static_transform_goal_base.transform = tf2::toMsg(transformed_goal_base);

        geometry_msgs::PoseStamped pose_got;
        pose_got.header.frame_id="world";
        pose_got.header.stamp = ros::Time::now();
        pose_got.pose.position.x = static_transform_goal_base.transform.translation.x;
        pose_got.pose.position.y = static_transform_goal_base.transform.translation.y;
        pose_got.pose.position.z = static_transform_goal_base.transform.translation.z;
        pose_got.pose.orientation = tf2::toMsg(new_quat);
        // pose_got.pose.orientation.x = static_transform_goal_base.transform.rotation.x;
        // pose_got.pose.orientation.y = static_transform_goal_base.transform.rotation.y;
        // pose_got.pose.orientation.z = static_transform_goal_base.transform.rotation.z;
        // pose_got.pose.orientation.w = static_transform_goal_base.transform.rotation.w;
        // std::cout << pose_got << std::endl;
        previous_quat = new_quat;

        // check pose
        // geometry_msgs::PoseStamped pose_check;
        // pose_check.header.frame_id="world";
        // pose_check.header.stamp = ros::Time::now();
        // pose_check.pose.position.x = static_transform_goal_base.transform.translation.x;
        // pose_check.pose.position.y = static_transform_goal_base.transform.translation.y;
        // pose_check.pose.position.z = static_transform_goal_base.transform.translation.z;
        // pose_check.pose.orientation.x = static_transform_goal_base.transform.rotation.x;
        // pose_check.pose.orientation  = tf2::toMsg(new_quat);

        // pub_check.publish(pose_check);

        return pose_got;

        


    }
};
        
int main(int argc, char **argv)
{
    ros::init(argc,argv, "contact_force_controller");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<tera_iiwa_ros::ForceZConfig> server;
    dynamic_reconfigure::Server<tera_iiwa_ros::ForceZConfig>::CallbackType f;
    ForceController fc = ForceController(&nh);
    f = boost::bind(&ForceController::callback_dyn_param, fc, _1, _2);
    server.setCallback(f);
    ros::spin();
}
