#!/usr/bin/env python

from __future__ import print_function

import threading

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

import sys
from select import select


import termios
import tty

PoseMsg = PoseStamped

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        # 'i':(1,0,0,0),
        # 'o':(1,0,0,-1),
        # 'j':(0,0,0,1),
        # 'l':(0,0,0,-1),
        # 'u':(1,0,0,1),
        # ',':(-1,0,0,0),
        # '.':(-1,0,0,1),
        # 'm':(-1,0,0,-1),
        # 'O':(1,-1,0,0),
        # 'I':(1,0,0,0),
        # 'J':(0,1,0,0),
        # 'L':(0,-1,0,0),
        # 'U':(1,1,0,0),
        # '<':(-1,0,0,0),
        # '>':(-1,-1,0,0),
        # 'M':(-1,1,0,0),
        # 't':(0,0,1,0),
        # 'b':(0,0,-1,0),
        '8': (1,0,0,0,0,0,0),
        '2': (-1,0,0,0,0,0,0),
        '4': (0,1,0,0,0,0,0),
        '6': (0,-1,0,0,0,0,0),
        '5': (0,0,1,0,0,0,0),
        '0': (0,0,-1,0,0,0,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/cartesian_trajectory_generator/new_goal', PoseMsg, queue_size = 1)
        self.x = trans_desired[0]
        self.y = trans_desired[1]
        self.z = trans_desired[2]
        self.xr = rot_desired[0]
        self.yr = rot_desired[1]
        self.zr = rot_desired[2]
        self.wr = rot_desired[3]
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, xr, yr, zr, wr):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.xr = xr
        self.yr = yr
        self.zr = zr
        self.wr = wr
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(trans_desired[0],trans_desired[1],trans_desired[2],
                    rot_desired[0],rot_desired[1],rot_desired[2],rot_desired[3])
        self.join()

    def run(self):
        pose_msg = PoseMsg()
        pose = pose_msg.pose
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = _frame

        while not self.done:
            pose_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into pose message.
            pose.position.x = self.x 
            pose.position.y = self.y 
            pose.position.z = self.z
            pose.orientation.x = self.xr
            pose.orientation.y = self.yr
            pose.orientation.z = self.zr
            pose.orientation.w = self.wr

            self.condition.release()

            # Publish.
            self.publisher.publish(pose_msg)

        # Publish stop message when thread exits.
        pose.position.x = -0.62 
        pose.position.y = 0.0
        pose.position.z = 0.15
        pose.orientation.x = 0.7
        pose.orientation.y = 0.7
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        self.publisher.publish(pose_msg)


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

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# def vels(speed, turn):
#     return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')
    # tf listener
    listener = tf.TransformListener()
    # speed = rospy.get_param("~speed", 0.5)
    # turn = rospy.get_param("~turn", 1.0)
    # speed_limit = rospy.get_param("~speed_limit", 1000)
    # turn_limit = rospy.get_param("~turn_limit", 1000)
    # repeat = rospy.get_param("~repeat_rate", 0.0)
    # key_timeout = rospy.get_param("~key_timeout", 0.5)
    # stamped = rospy.get_param("~stamped", True)
    # _frame = rospy.get_param("~frame_id", 'world')
    _frame = "world"
    PoseMsg = PoseStamped

    pub_thread = PublishThread(0.0) #repeat

    
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        # pub_thread.update(x, y, z, xr, yr, zr, wr)

        print(msg)
        # print(vels(speed,turn))
        while(1):
            good = False
            while not good:
                try:
                    # from iiwa_link_0 to iiwa_link_ee
                    (trans_desired, rot_desired) = listener.lookupTransform('/iiwa_link_0', '/tool_link_ee', rospy.Time(0))
                    good = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.sleep(1.)
            x = trans_desired[0]
            y = trans_desired[1]
            z = trans_desired[2]
            xr = trans_desired[0]
            yr = rot_desired[1]
            zr = rot_desired[2]
            wr = rot_desired[3]
            key = getKey(settings, 0.5)#key_timeout
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                # xr = moveBindings[key][3]
            # elif key in speedBindings.keys():
            #     speed = min(speed_limit, speed * speedBindings[key][0])
            #     turn = min(turn_limit, turn * speedBindings[key][1])
            #     if speed == speed_limit:
            #         print("Linear speed limit reached!")
            #     if turn == turn_limit:
            #         print("Angular speed limit reached!")
            #     print(vels(speed,turn))
            #     if (status == 14):
            #         print(msg)
            #     status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == -0.62 and y == 0 and z == 0.15:
                    continue
                x = -0.62
                y = 0
                z = 0.15
                xr = 0.7
                yr = 0.7
                zr = 0
                wr = 0              # th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, xr, yr, zr, wr)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)