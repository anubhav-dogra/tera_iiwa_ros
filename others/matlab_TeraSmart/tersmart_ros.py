import rospy
from geometry_msgs.msg import Wrench

import socket
ip_addr = "137.205.241.215"
port = 8001
start_time = -340
end_time = -290
start_command  = "START"
status_command = "GETSATUS"
starttime_command = "SETSTARTVALUE"
endtime_command = "SETENDVALUE"
setmode_commnad = "SETMODE"
numpoints_command = "GETNUMBEROFPOINTS"
time_command= "GETTIMEAXIS"
get_pulse_command = "GETLATESTPULSE"
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect((ip_addr,port))
sock.sendall(start_command.encode('utf-8'))
return_value = sock.recv(8)
print(return_value)
sock.sendall(time_command.encode('utf-8'))
numpoints = sock.recv(8)
print(numpoints)

sock.close()

def CallbackWrench(msg:Wrench):
    global prev_Fz, current_Fz,counter
    current_Fz = msg.force.z
    if counter == 0:
        prev_Fz = current_Fz
        print(prev_Fz)
        counter +=1
    if abs(current_Fz - prev_Fz)>=3:
        print("THz is recording")
        print(current_Fz)

    



if __name__=="__main__":
    global prev_Fz, current_Fz, counter
    counter = 0
    prev_Fz = 0
    current_Fz = 0
    rospy.init_node("terasmart_ros")
    #rospy.wait_for_message("/cartesian_wrench")
    rospy.Subscriber("/cartesian_wrench",Wrench,callback=CallbackWrench)

    rospy.spin()