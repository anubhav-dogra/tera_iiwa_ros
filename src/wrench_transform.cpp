#include "ros/ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class Transform_things
{
public:
    Transform_things():
    tf2_(buffer_), target_frame_("tool_link_ee"),
    tf2_filter_(point_sub_, buffer_, target_frame_,10,0)
{
    point_sub_.subscribe(n_,"cartesian_wrench",10);
    tf2_filter_.registerCallback(boost::bind(&Transform_things::msgCallback, this, _1));

}

void msgCallback(const geometry_msgs::WrenchStampedConstPtr wrench_ptr)
{
    geometry_msgs::WrenchStamped wrench_new;
    wrench_new.header = wrench_ptr->header;
    wrench_new.header.frame_id = target_frame_;
    
    try{
    geometry_msgs::TransformStamped transformation;
    transformation = buffer_.lookupTransform(target_frame_, 
                                                  wrench_ptr->header.frame_id, 
                                                  wrench_ptr->header.stamp);
    // Apply transformation to force and torque vectors                                         
     tf2::doTransform(wrench_ptr->wrench.force, wrench_new.wrench.force, transformation);
     tf2::doTransform(wrench_ptr->wrench.torque, wrench_new.wrench.torque, transformation);
    // Publish transformed wrench.
    // ros::Duration(0.25).sleep();
    pub.publish(wrench_new);
    pub1.publish(transformation);
    }
    
    catch (tf2::TransformException &ex){
            ROS_WARN("Failure to transform wrench data: %s\n", ex.what());
        }
    
}
private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_;
    ros::NodeHandle n_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> point_sub_;
    tf2_ros::MessageFilter<geometry_msgs::WrenchStamped> tf2_filter_;
    ros::Publisher pub = n_.advertise<geometry_msgs::WrenchStamped>("/cartesian_wrench_tool", 10);
    ros::Publisher pub1 = n_.advertise<geometry_msgs::TransformStamped>("/tool_link_ee_pose", 10);
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wrench_transform");
    Transform_things Tt;
    ros::spin();
    return 0;
}
