import rospy
from geometry_msgs.msg import Wrench

import socket
ip_addr = "137.205.241.215"
port = 8001
command  = "STOP"
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect((ip_addr,port))
sock.sendall(command.encode('utf-8'))
return_value = sock.recv(8)
print(return_value)
sock.close()

# def CallbackWrench(msg:Wrench):
#     global Fz
#     Fz = msg.force.z


# if __name__=="__main__":
#     global Fz
#     rospy.init_node("terasmart_ros")
#     rospy.wait_for_message("/cartesian_wrench")
#     rospy.Subscriber("/cartesian_wrench",Wrench,callback=CallbackWrench)

#     rospy.spin()