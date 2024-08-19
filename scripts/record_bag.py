import rospy
import rosbag
from geometry_msgs.msg import WrenchStamped, TransformStamped
import os, sys


class RecordBag:
    def __init__(self, filename):
        # get home directory
        home = os.path.expanduser("~")
        self.bag = rosbag.Bag(home + '/Recordings/bags/' +  filename, 'w')
        self.ee_pose_sub = rospy.Subscriber('/tool_link_ee_pose', TransformStamped, self.ee_pose_callback, queue_size=10)
        self.wrench_tool_sub = rospy.Subscriber('/cartesian_wrench_tool', WrenchStamped, self.wrench_tool_callback, queue_size=10)


    def __del__(self):
        self.bag.close()

    def ee_pose_callback(self, msg):
        rospy.loginfo("Received ee_pose message")
        self.bag.write('/tool_link_ee_pose', msg)

    def wrench_tool_callback(self, msg):
        rospy.loginfo("Received wrench_tool message")
        self.bag.write('/cartesian_wrench_tool', msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("record_bag", anonymous=True)
    # filename = 'test.bag'
    filename = sys.argv[1] + '.bag'
    # topic_names = ['/tool_link_ee_pose', '/cartesian_wrench_tool_ts']
    
    record_bags = RecordBag(filename)
    try:
        record_bags.run()
    except rospy.ROSInterruptException:
        # record_bags.__del__()
        pass
    finally:
        # record_bags.__del__()
        rospy.loginfo("Recording stopped and bag file closed")