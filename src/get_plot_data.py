import rospy
from iiwa_driver.msg import AdditionalOutputs

def iiwa_output_callback(incoming_msg):
    rospy.loginfo(incoming_msg.commanded_torques)

def listener():
    rospy.init_node("get_plot_data_py", anonymous=True)
    rospy.Subscriber("/additional_outputs", AdditionalOutputs, iiwa_output_callback)
    rospy.spin()


if __name__== '__main__':
    listener()