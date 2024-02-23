import rospy
from sensor_msgs.msg import JointState
from cartesian_impedance_controller.msg import ControllerConfig
import numpy as np
from iiwa_tools.srv import GetJacobian
from scipy.optimize import minimize

def cal_manipulability(joint_position):
    ## Get Jacobian from the jacobian server
    jac_service = "/iiwa/iiwa_jacobian_server"
    rospy.wait_for_service(jac_service)
    good = False
    while not good:
        try:
            # Create a proxy for the service
            get_jac = rospy.ServiceProxy(jac_service, GetJacobian)
            jac_resp = get_jac(joint_angles = joint_position, joint_velocities=joint_velocities)        
            jac_out = jac_resp.jacobian
            jac_array = jac_out.data
            jac_layout = jac_out.layout
            jac_dim_sizes = [dim.size for dim in jac_layout.dim]
            J = np.array(jac_array).reshape(jac_dim_sizes)
            # print(J)
            good = True
        except rospy.ServiceException as e:
            print("Service call failed: %s" %e)
    
    # Assuming 'J' is the Jacobian matrix with shape (6, 7)
    # Calculate the product of J and its transpose J^T
    JJT = np.dot(J, J.T)
    # Alternatively, you can use JJT = J @ J.T

    manipulability = np.sqrt(np.linalg.det(JJT))
    return -manipulability
    # print(manipulability)


def callback_joints(data):
    global joint_velocities
    joint_position = data.position[0:7]
    joint_velocities = data.velocity[0:7]
    # joint_velocities = [0,0,0,0,0,0,0]
    m = cal_manipulability(joint_position)
    print(m)

    ## Optimize manipulability
    # initial_guess = joint_position
    # print("initial_guess",initial_guess)

    # opt_result = minimize(cal_manipulability,initial_guess)
    # optimal_joint_configuration = opt_result.x
    # print(optimal_joint_configuration)



    ## check nullspace command sending. 
    # factor = -0.01
    # joint_values = [joint_position[0]+factor,
    #                 joint_position[1]+factor,
    #                 joint_position[2]+factor,
    #                 joint_position[3],
    #                 joint_position[4]+factor,
    #                 joint_position[5]+factor,
    #                 joint_position[6]+factor]
    
    # controller_config = set_controller_config(joint_values)
    # print("controller_config", controller_config.q_d_nullspace)
    # pub_controller_config.publish(controller_config)
    
def set_controller_config(joint_values):
    msg = ControllerConfig()

    msg.cartesian_stiffness.force.x = 400
    msg.cartesian_stiffness.force.y = 400
    msg.cartesian_stiffness.force.z = 400
    msg.cartesian_stiffness.torque.x = 50
    msg.cartesian_stiffness.torque.y = 50
    msg.cartesian_stiffness.torque.z = 50

    # Negative values mean that the default damping values apply --> 2* sqrt(stiffness)
    msg.cartesian_damping.force.x =  2*np.sqrt(msg.cartesian_stiffness.force.x)
    msg.cartesian_damping.force.y =  2*np.sqrt(msg.cartesian_stiffness.force.y)
    msg.cartesian_damping.force.z =  2*np.sqrt(msg.cartesian_stiffness.force.z)
    msg.cartesian_damping.torque.x = 2*np.sqrt(msg.cartesian_stiffness.torque.x)
    msg.cartesian_damping.torque.y = 2*np.sqrt(msg.cartesian_stiffness.torque.y)
    msg.cartesian_damping.torque.z = 2*np.sqrt(msg.cartesian_stiffness.torque.z)

    msg.q_d_nullspace.append(joint_values[0])
    msg.q_d_nullspace.append(joint_values[1])
    msg.q_d_nullspace.append(joint_values[2])
    msg.q_d_nullspace.append(joint_values[3])
    msg.q_d_nullspace.append(joint_values[4])
    msg.q_d_nullspace.append(joint_values[5])
    msg.q_d_nullspace.append(joint_values[6])
        

    msg.nullspace_stiffness = 5
    msg.nullspace_damping = 2*(np.sqrt(msg.nullspace_stiffness))
    return msg
    

if __name__=='__main__':
    rospy.init_node("nullspace_controller")
    # pub_controller_config = rospy.Publisher('/iiwa/CartesianImpedance_trajectory_controller/set_config', ControllerConfig, queue_size=10)
    
    ## if required in loop
    joint_sub = rospy.Subscriber('/iiwa/joint_states',JointState,callback_joints)
    rospy.spin()

    ## this run only once. 
    # msg_ = rospy.wait_for_message('/iiwa/joint_states',JointState)
    # callback_joints(msg_)


    # positions = msg_.position[0:7]
    # velocities = msg_.velocity[0:7]
    # factor = 0.1
    # print("joint_positions_now", msg_.position[0:7])
    # joint_values = [positions[0]+factor,
    #                 positions[1]+factor,
    #                 positions[2]+factor,
    #                 positions[3],
    #                 positions[4]+factor,
    #                 positions[5]+factor,
    #                 positions[6]+factor]
    # # joint_values = [0.0003,0.0005,0.0005,0,0,0,0]
    # controller_config = set_controller_config(joint_values)
    # print("controller_config", controller_config)
    # rospy.sleep(2)
    # pub_controller_config.publish(controller_config)
    # print("controller_config", controller_config)
    
    
    
    