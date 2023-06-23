import rospy
from rospy import Time
from tf2_ros import buffer
import tf2_ros 
from geometry_msgs.msg import PoseStamped, TransformStamped
import time
import tty, sys, termios
from select import select


pose_msg = PoseStamped()
# def keyboard_input():
msg = """
Reading from the keyboard  and Publishing to Cartesian_trajectory_generator/new_goal!
---------------------------
all poses are w.r.t. to the Base frame

8 : +x          4: (+y)
2 : -x          6: (-y)
5 : up (+z)
0 : down (-z)

q or CTRL-C to quit
"""

delta = 0.1
delta_r = 0.05
moveBindings = {
        '8': (delta,0,0,0,0,0,0),
        '2': (-delta,0,0,0,0,0,0),
        '4': (0,delta,0,0,0,0,0),
        '6': (0,-delta,0,0,0,0,0),
        '5': (0,0,0.01,0,0,0,0),
        '0': (0,0,-0.01,0,0,0,0),
        'w': (0,0,0,0,0,0,delta_r),
        'a': (0,0,0,delta_r,0,0,0),
        's': (0,0,0,0,delta_r,0,0),
        'd': (0,0,0,0,0,delta_r,0),
        'i': (0,0,0,0,0,0,-delta_r),
        'j': (0,0,0,-delta_r,0,0,0),
        'k': (0,0,0,0,-delta_r,0,0),
        'l': (0,0,0,0,0,-delta_r,0),
  
    }

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)
def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def update(x, y, z, xr, yr, zr, wr, tup):
        # self.condition.acquire()
        new_x = x   + tup[0]
        new_y = y   + tup[1]
        new_z = z   + tup[2]
        new_xr = xr + tup[3]
        new_yr = yr + tup[4]
        new_zr = zr + tup[5]
        new_wr = wr + tup[6]
        # Notify publish thread that we have a new message.
        # self.condition.notify()
        # self.condition.release()
        pose_msg.header.frame_id= "world"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = new_x
        pose_msg.pose.position.y = new_y
        pose_msg.pose.position.z = new_z
        pose_msg.pose.orientation.x = new_xr
        pose_msg.pose.orientation.y = new_yr
        pose_msg.pose.orientation.z = new_zr
        pose_msg.pose.orientation.w = new_wr

        return pose_msg

    
# def wait_for_subscribers(self):
#     i = 0
#     while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
#         if i == 4:
#             print("Waiting for subscriber to connect to {}".format(self.publisher.name))
#         rospy.sleep(0.5)
#         i += 1
#         i = i % 5
#     if rospy.is_shutdown():
#         raise Exception("Got shutdown request before subscribers connected")

def teleoperator():
    rospy.init_node('teleoperator', anonymous=True)
    rospy.Rate(200)
    buffer_ = tf2_ros.Buffer()
    listerner = tf2_ros.TransformListener(buffer_)
    publish_ = rospy.Publisher("cartesian_trajectory_generator/new_goal", PoseStamped, queue_size=1)
    # pub1 = rospy.Publisher("tool_link_ee_pose_", TransformStamped, queue_size=1)  
    while not rospy.is_shutdown():      
        try:
            now = rospy.Time.now()
            transformation = buffer_.lookup_transform("iiwa_link_0", "tool_link_ee", rospy.Time(0), timeout=rospy.Duration(2))
            # print(transformation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # rospy.sleep(1.)
            print("exception occured")  

        _x = transformation.transform.translation.x
        _y = transformation.transform.translation.y
        _z = transformation.transform.translation.z
        _xr = transformation.transform.rotation.x
        _yr = transformation.transform.rotation.y
        _zr = transformation.transform.rotation.z
        _wr = transformation.transform.rotation.w
        # pub1.publish(transformation)


        # print(_x,_y,_z,_xr,_yr,_zr,_wr)
        key = getKey(settings, 1)
        if key in moveBindings.keys():
            new_pose = update(_x, _y, _z, _xr, _yr, _zr, _wr, moveBindings[key])
            publish_.publish(new_pose)
                
        elif (key == '\x03'):
            break
        elif (key == 'q'):
            break

    restoreTerminalSettings(settings)    


if __name__ == '__main__':
    settings = saveTerminalSettings()

    try:
        teleoperator()
    except rospy.ROSInterruptException:
        pass