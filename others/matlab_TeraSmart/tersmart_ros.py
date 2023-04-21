# import rospy
# from geometry_msgs.msg import Wrench

import socket, struct
ip_addr = "137.205.241.215"
port = 8001
start_time = -340
end_time = -290

def submit(command, type_id, n_bit):
    sock = socket.socket()
    sock.connect((ip_addr,port))
    sock.sendall(command.encode('utf-8'))
    return_value = [e(0) for e in list(struct.iter_unpack(type_id, sock.recv(n_bit)))]
    sock.close()
    return return_value

return_value = submit("START","?", 8)#Starting/Stopping Spectrometer if scanning
return_value = submit("GETSTATUS","i",32)#getting the status
return_value = submit("SETSTARTVALUE -340", "?", 8)#setting the start of the scan
return_value = submit("SETENDVALUE -290", "?", 8)#setting the end of the scan
return_value = submit("START","?", 8)#StartingSpectrometer if scanning
numpoints = submit("GETNUMBEROFPOINTS", "i", 32) #number of points of the pulse
print(numpoints[0])
pulse_data = submit("GETLATESTPULSE", "d", 8*numpoints[0]) #array with the pulse data
time_axis = submit("GETTIMEAXIS", "d", 8*numpoints[0]) #array with the time data
print("unfinished")


# def CallbackWrench(msg:Wrench):
#     global prev_Fz, current_Fz,counter
#     current_Fz = msg.force.z
#     if counter == 0:
#         prev_Fz = current_Fz
#         print(prev_Fz)
#         counter +=1
#     if abs(current_Fz - prev_Fz)>=3:
#         print("THz is recording")
#         print(current_Fz)

    



# if __name__=="__main__":
#     global prev_Fz, current_Fz, counter
#     counter = 0
#     prev_Fz = 0
#     current_Fz = 0
#     rospy.init_node("terasmart_ros")
#     #rospy.wait_for_message("/cartesian_wrench")
#     rospy.Subscriber("/cartesian_wrench",Wrench,callback=CallbackWrench)

#     rospy.spin()