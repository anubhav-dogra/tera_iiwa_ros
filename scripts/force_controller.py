import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped
import message_filters
import tf2_ros

class ForceController:
    def __init__(self) -> None:
        
        self.Desired_force = 3 # 10 N desired force to applied
        self.Kp = 1
        self.Kd = 1
        self.Kf = 1000
        self.prev_error_fz = 0
        self.rate = rospy.Rate(10)
        self.dt = 1/10
        self.curr_pose = PoseStamped()
        self.error_fz = 0
        self.init_pose  = PoseStamped()
        self.init_pose.header.frame_id = "world"
        self.init_pose.header.stamp = rospy.Time.now()
        # self.init_pose.pose.position.x = -0.62
        # self.init_pose.pose.position.y = 0.0
        # self.init_pose.pose.position.z = 0.2
        # self.init_pose.pose.orientation.x = 0.7
        # self.init_pose.pose.orientation.y = 0.7
        # self.init_pose.pose.orientation.z = 0.0
        # self.init_pose.pose.orientation.w = 0.0
        buffer_ = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer_)
        try:
            now = rospy.Time.now()
            transformation = buffer_.lookup_transform("iiwa_link_0", "tool_link_ee", rospy.Time(0), timeout=rospy.Duration(2))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("exception occured")   

        self.init_pose.pose.position.x = transformation.transform.translation.x
        self.init_pose.pose.position.y = transformation.transform.translation.y
        self.init_pose.pose.position.z = transformation.transform.translation.z
        self.init_pose.pose.orientation.x = transformation.transform.rotation.x
        self.init_pose.pose.orientation.y = transformation.transform.rotation.y
        self.init_pose.pose.orientation.z = transformation.transform.rotation.z
        self.init_pose.pose.orientation.w = transformation.transform.rotation.w
        print(self.init_pose)

        self.pose_sub = message_filters.Subscriber("/tool_link_ee_pose", TransformStamped)    
        self.force_sub = message_filters.Subscriber("/cartesian_wrench_tool", WrenchStamped)
        # self.force_sub = message_filters.Subscriber("/ft_sensor/raw", WrenchStamped)
        # self.pose_pub = rospy.Publisher("/cartesian_trajectory_generator/new_goal", PoseStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/iiwa/CartesianImpedance_trajectory_controller/reference_pose", PoseStamped, queue_size=1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.force_sub], 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)


    # def current_pose(self, pose_msg):
    #     self.curr_pose.header.frame_id = "world"
    #     self.curr_pose.header.stamp = rospy.Time.now()
    #     self.curr_pose.pose.position.x = pose_msg.transform.translation.x
    #     self.curr_pose.pose.position.y = pose_msg.transform.translation.y
    #     self.curr_pose.pose.position.z = pose_msg.transform.translation.z
    #     self.curr_pose.pose.orientation.x = pose_msg.transform.rotation.x
    #     self.curr_pose.pose.orientation.y = pose_msg.transform.rotation.y
    #     self.curr_pose.pose.orientation.z = pose_msg.transform.rotation.z
    #     self.curr_pose.pose.orientation.w = pose_msg.transform.rotation.w



    def callback(self, pose_msg, force_msg):
        # self.rate.sleep()
        # self.curr_pose.header.frame_id = "world"
        # self.curr_pose.header.stamp = rospy.Time.now()
        # self.curr_pose.pose.position.x = pose_msg.transform.translation.x
        # self.curr_pose.pose.position.y = pose_msg.transform.translation.y
        # self.curr_pose.pose.position.z = pose_msg.transform.translation.z
        # self.curr_pose.pose.orientation.x = pose_msg.transform.rotation.x
        # self.curr_pose.pose.orientation.y = pose_msg.transform.rotation.y
        # self.curr_pose.pose.orientation.z = pose_msg.transform.rotation.z
        # self.curr_pose.pose.orientation.w = pose_msg.transform.rotation.w

        
        self.curr_force = force_msg.wrench.force.z

        print("current_force", self.curr_force)
        self.error_fz = self.Desired_force - abs(self.curr_force)
        print("self.error_fz", self.error_fz)
        dFe = self.error_fz - self.prev_error_fz
        # print("dFe", dFe)
        # print("prev_error", self.prev_error_fz)
        self.prev_error_fz = self.error_fz
        # dZ = (self.Kp*self.error_fz + self.Kd*dFe)*self.dt
        dZ = (self.Kp/self.Kf)*self.error_fz*self.dt + (self.Kd/self.Kf)*dFe*self.dt
        # print("dZ",dZ)
        self.update_pose(dZ)    
  
    
    def update_pose(self, dZ):
        # if (self.curr_force < 10):
        # print("poseinZ_before",self.init_pose.pose.position.z)
        self.init_pose.pose.position.z -= dZ
        # print("poseinZ",self.init_pose.pose.position.z)
        # print("dZ_again", dZ)
        # self.init_pose.pose.position.x = -0.62
        # self.init_pose.pose.position.y = 0.0
        # self.init_pose.pose.orientation.x = 0.7
        # self.init_pose.pose.orientation.y = 0.7
        # self.init_pose.pose.orientation.z = 0.0
        # self.init_pose.pose.orientation.w = 0.0
        
        # print(self.init_pose)
        self.pose_pub.publish(self.init_pose)
        # rospy.sleep(0.5)

        # if (self.curr_force > 10):
        #     self.init_pose.pose.position.z += dZ
        #     print(self.init_pose.pose.position.z)
        #     self.pose_pub.publish(self.init_pose)
            # rospy.sleep(0.5)
    
         

if __name__ == '__main__':
    rospy.init_node("force_controller", anonymous=True)

    ForceController()
    rospy.spin()

    
