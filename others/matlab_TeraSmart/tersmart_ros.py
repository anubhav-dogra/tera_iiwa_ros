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

def CallbackWrench(msg:Wrench):
    global prev_Fz, current_Fz,counter
    current_Fz = msg.force.z
    if counter == 0:
        prev_Fz = current_Fz
        counter +=1
    if abs(current_Fz - prev_Fz)>=3:
        print("THz is recording")

    



if __name__=="__main__":
    global prev_Fz, current_Fz, counter
    counter = 0
    prev_Fz = 0
    current_Fz = 0
    rospy.init_node("terasmart_ros")
    rospy.wait_for_message("/cartesian_wrench")
    rospy.Subscriber("/cartesian_wrench",Wrench,callback=CallbackWrench)

    rospy.spin()