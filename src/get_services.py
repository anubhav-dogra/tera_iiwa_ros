import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
from iiwa_tools.srv import GetIK, GetFK
import numpy as np
from angles import shortest_angular_distance, shortest_angular_distance_with_limits

# Array/list to read joint values from robot
joints = [0, 0, 0, 0, 0, 0, 0]

# Callback for reading joint values
def joint_callback(data):
    global joints
    joints = data.position

# Initialize ROS node
rospy.init_node('test')

# Let's make a publisher to control the robot
pub = rospy.Publisher('/iiwa/PositionController/command', Float64MultiArray, queue_size=10)
# Subscribe to /iiwa/joint_states with a callback (joint_callback)
rospy.Subscriber("/iiwa/joint_states", JointState, joint_callback)

# Let's create a message for command sending
cmd_msg = Float64MultiArray()

# tf listener
listener = tf.TransformListener()

# Names of IK/FK services
ik_service = '/iiwa/iiwa_ik_server'
fk_service = '/iiwa/iiwa_fk_server'

# Wait for services to spawn (it might take a while)
rospy.wait_for_service(ik_service)
rospy.wait_for_service(fk_service)

# Read current pose to set it as target
# CAUTION: This is a dummy valid target. Set your translation/rotation to your desired target!
trans_desired = [0, 0, 0]
rot_desired = [0, 0, 0, 0]

good = False
while not good:
    try:
        # from iiwa_link_0 to iiwa_link_ee
        (trans_desired, rot_desired) = listener.lookupTransform('/iiwa_link_ee', '/iiwa_link_0', rospy.Time(0))
        good = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.sleep(1.)

# Query IK to go to that location
pose_msg = Pose()
pose_msg.position.x = trans_desired[0]
pose_msg.position.y = trans_desired[1]
pose_msg.position.z = trans_desired[2]
pose_msg.orientation.x = rot_desired[0]
pose_msg.orientation.y = rot_desired[1]
pose_msg.orientation.z = rot_desired[2]
pose_msg.orientation.w = rot_desired[3]

# Print for debugging
# print(pose_msg)

solution = []
good = False
while not good:
    # call the IK service
    try:
        get_ik = rospy.ServiceProxy(ik_service, GetIK)
        seed = Float64MultiArray()
        seed.layout = MultiArrayLayout()
        seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        seed.layout.dim[0].size = 1
        seed.layout.dim[1].size = 7
        # This is not necessarily needed, it gives an initial guess of the possible joint solution
        seed.data = [0., 1., 0., -1., 0., 1., 0.]

        # Let's get the result from IK
        resp1 = get_ik(poses=[pose_msg], seed_angles=seed)
        solution = resp1.joints.data
        good = True
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# # Let's call FK with the solution to check if it is a good one; just for verification
# good = False
# while not good:
#     # call the FK service
#     try:
#         get_fk = rospy.ServiceProxy(fk_service, GetFK)
#         seed = Float64MultiArray()
#         seed.layout = MultiArrayLayout()
#         seed.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
#         seed.layout.dim[0].size = 1
#         seed.layout.dim[1].size = 7
#         seed.data = solution
#         resp1 = get_fk(joints=seed)
#         sol_pose = resp1.poses[0]
#         print('sol:', sol_pose)
#         good = True
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

print('IK solution:', solution)
target = solution

# Robot limits
lower_limits = -np.array([170, 120, 170, 120, 170, 120 ,175])
lower_limits = lower_limits*np.pi/180.
upper_limits = np.array([170, 120, 170, 120, 170, 120 ,175])
upper_limits = upper_limits*np.pi/180.

# Control the robot to go the computed position (This assumes that the robot is in position control)
while not rospy.is_shutdown():
    # copy global joints into temporal variable
    jt = joints

    # Compute differences of current joint positions to the acquired from IK solution
    diff = []
    for i in range(len(target)):
        diff.append(shortest_angular_distance_with_limits(jt[i], target[i], lower_limits[i], upper_limits[i])[1])

    # Small trick to avoid noisy or very fast movements
    min_threshold = 1e-3
    max_dist = 0.03
    for i in range(len(diff)):
        # Do not allow very big joint commands
        if diff[i] > max_dist:
            diff[i] = max_dist
        elif diff[i] < -max_dist:
            diff[i] = -max_dist
        # Do not allow too small joint commands
        if np.abs(diff[i]) < min_threshold:
            diff[i] = 0

    # Joint position target: current joint + modified differences
    joint_target = np.array(jt) + np.array(diff)

    # Send the data to the robot to follow
    cmd_msg.data = [joint_target[0], joint_target[1], joint_target[2], joint_target[3], joint_target[4], joint_target[5], joint_target[6]]
    pub.publish(cmd_msg)
    rospy.sleep(0.005)

rospy.spin()