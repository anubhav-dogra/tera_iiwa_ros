#include "ros/ros.h"
#include "iiwa_driver/AdditionalOutputs.h"
#include <iiwa_tools/GetFK.h>
#include <iiwa_tools/GetJacobian.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
//#include <tf/tf.h>
#include <geometry_msgs/Wrench.h>

double joint_positions_[6];
double joint_velocities_[6];
Eigen::VectorXd ext_torques(7);
int n;
using namespace std;
Eigen::MatrixXd J(6,7);
Eigen::MatrixXd Ext_torq(7,1);
geometry_msgs::Wrench eef_wrench;

ros::ServiceClient *fk_clientPtr;
ros::ServiceClient *J_clientPtr;
ros::Publisher pub_ee_wrench;
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}   
    

// Helper function to set value in a MultiArray
void set_multi_array(std_msgs::Float64MultiArray& array, size_t i, size_t j, double val)
{
    assert(array.layout.dim.size() == 2);
    size_t offset = array.layout.data_offset;

    array.data[offset + i * array.layout.dim[0].stride + j] = val;
}
//void iiwa_output_callback(iiwa_driver::AdditionalOutputs incoming_msg){
void iiwa_output_callback(std_msgs::Float64MultiArray incoming_msg){
    //cout<< "im in" <<endl;
    std::vector<double> command_tor = incoming_msg.data;
    // for (int i = 0; i < ext_torques.size(); i++)
    // {
    //    std::cout << "i:::" << i<< command_tor[i] << std::endl;
       
    //    //ext_torques[i] = command_tor[i];
    // }
    int count=0;
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 1; j++)
        {
         Ext_torq(i,j) = command_tor[count];
         
         //std::cout<< i << ":::"<< j << ":::"<< J(i,j) << std::endl;
         count++;
         
        }
        
    }
    
        
   
    //std::cout << Ext_torq << std::endl;
    const Eigen::MatrixXd J_t_pinv = pseudo_inverse(J.transpose());
    
    Eigen::VectorXd external_ee_wrench = J_t_pinv*Ext_torq;
    //tf::wrenchEigenToMsg(external_ee_wrench, eef_wrench);
    // Eigen::VectorXd external_ee_wrench = J*ext_torques;
    cout << "eef_wrench=" << external_ee_wrench << endl;
    //pub_ee_wrench.publish()
    //std::cout << ext_torques << std::endl;
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
        joint_positions_[i] = inflow_j_states.position[i];
        joint_velocities_[i] = inflow_j_states.velocity[i];
         //std::cout << joint_positions[i] << std::endl;
    
    } 
    // // required for Inputting joint_states for GetFK
    // std_msgs::Float64MultiArray positions_joints;
    // positions_joints.layout.dim.resize(2);
    // positions_joints.layout.data_offset = 0;
    // positions_joints.layout.dim[0].size = 1;
    // positions_joints.layout.dim[0].stride = 7;
    // positions_joints.layout.dim[1].size = 7;
    // positions_joints.layout.dim[1].stride = 0;
    // positions_joints.data.resize(1*7);
    // for (int i = 0; i < n; ++i)
    // {
    //     positions_joints.data[i] = joint_positions[i];     
    // }
    // std::cout <<  positions_joints << std::endl;

    // iiwa_tools::GetFK get_fk_srv;
    //ROS_INFO("calling the service");
    // get_fk_srv.request.joints = positions_joints;
    // if (fk_clientPtr->call(get_fk_srv))
    // {
    //    ROS_INFO("The iiwa_ee_state X: %f Y: %f, Z: %f", get_fk_srv.response.poses[0].position.x, get_fk_srv.response.poses[0].position.y, get_fk_srv.response.poses[0].position.z);
    //    ROS_INFO("The iiwa_ee_state X.O: %f Y.O: %f, Z.O: %f, W.O: %f", get_fk_srv.response.poses[0].orientation.x, get_fk_srv.response.poses[0].orientation.y, get_fk_srv.response.poses[0].orientation.z, get_fk_srv.response.poses[0].orientation.w);
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service iiwa_fk_server");
    // }
    iiwa_tools::GetJacobian get_J_srv;
    for (int i = 0; i < n; ++i)
    {
            
    get_J_srv.request.joint_angles.push_back(joint_positions_[i]);
    get_J_srv.request.joint_velocities.push_back(joint_velocities_[i]);
    }
   
    if (J_clientPtr->call(get_J_srv))
    {
       //ROS_INFO("Jacobian: " get_J_srv.response.jacobian.data[0]);
       //std::cout << get_J_srv.response.jacobian << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service iiwa_jacobian_server");
    }
    std::vector<double> j_data = get_J_srv.response.jacobian.data;
    int count = 0;
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 7; j++)
        {
         J(i,j) = j_data[count];
         
         //std::cout<< i << ":::"<< j << ":::"<< J(i,j) << std::endl;
         count++;
         
        }
        
    }
    //cout << J << endl;
    
    
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
    ros::ServiceClient J_client = nh.serviceClient<iiwa_tools::GetJacobian>("/iiwa/iiwa_jacobian_server");
    J_clientPtr = &J_client;
    J_client.waitForExistence();
    ros::Subscriber sub_joint_states = nh.subscribe("/iiwa/joint_states", 100, iiwa_jointstates_callback);
    ros::Subscriber sub_add = nh.subscribe("/iiwa/CartesianImpedance_trajectory_controller/commanded_torques", 100, iiwa_output_callback);



    ros::waitForShutdown();

}