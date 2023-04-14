#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseDrawer
{
public:
    PoseDrawer() :
    tf2_(buffer_),  target_frame_("iiwa_link_0"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    // point_sub_.subscribe(n_, "pose_output", 10);
    point_sub_.subscribe(n_, "final_pose_output", 10);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
    
  }

  void msgCallback(const geometry_msgs::PoseArrayConstPtr& point_ptr) 
  {
    //std::cout<<"Im in"<<std::endl;
    geometry_msgs::PoseArray tf_array_out;
    tf_array_out.header.stamp = ros::Time::now();
    tf_array_out.header.frame_id = "iiwa_link_0";
    geometry_msgs::PoseStamped tf_output_pose;
    tf_output_pose.header.stamp = ros::Time::now();
    tf_output_pose.header.frame_id = "iiwa_link_0";
    geometry_msgs::PoseStamped point_now;
    point_now.header.stamp = ros::Time::now();
    //point_now.header.frame_id = "camera_image_link";
    point_now.header.frame_id = "camera_color_optical_frame";
    //std::cout<<point_ptr->poses.size()<< std::endl;//

    int array_size = point_ptr->poses.size();
    for(int i = 0; i < 1; i++) //array_size
      { 
        point_now.pose.position.x = point_ptr->poses[i].position.x;
        point_now.pose.position.y = point_ptr->poses[i].position.y;
        point_now.pose.position.z = point_ptr->poses[i].position.z;
        point_now.pose.orientation.x = point_ptr->poses[i].orientation.x;
        point_now.pose.orientation.y = point_ptr->poses[i].orientation.y;
        point_now.pose.orientation.z = point_ptr->poses[i].orientation.z;
        point_now.pose.orientation.w = point_ptr->poses[i].orientation.w;
        //std::cout<<point_now<< std::endl;
        try 
          {
          buffer_.transform(point_now, tf_output_pose, target_frame_);
          
          //std::cout<<tf_output_pose<<std::endl;
          //ROS_INFO(tf_output_pose
                  //tf_output_pose.point.x,
                  //tf_output_pose.point.y,
                  //tf_output_pose.point.z);
          }
        catch (tf2::TransformException &ex) 
        {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }
        // push in array
        //std::cout<<tf_output_pose<< std::endl;
        tf_array_out.poses.push_back(tf_output_pose.pose);
        std::cout<<"value is"<<tf_array_out.poses[0]<<std::endl;
      }
      

  pub.publish(tf_array_out);
 }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PoseArray> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseArray> tf2_filter_;
  ros::Publisher pub = n_.advertise<geometry_msgs::PoseArray>("tf_array_out", 10);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_listener_test"); //Init ROS
  PoseDrawer pd; //Construct class
  
  ros::spin(); // Run until interupted 
  return 0;
};
