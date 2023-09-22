import rospy
import sys
from geometry_msgs.msg import WrenchStamped
import socket, struct
import matplotlib.pyplot as plt


def submit(command, type_id, n_bit):
    sock = socket.socket()
    sock.connect((ip_addr,port))
    sock.sendall(command.encode('utf-8'))
    return_value = [e[0] for e in list(struct.iter_unpack(type_id, sock.recv(n_bit)))]
    sock.close()
    return return_value

def CallbackWrench(msg):
    global prev_Fz, current_Fz, counter, counter_, t1, numpoints
    current_Fz = msg.wrench.force.z
    if counter_ == 0:
        prev_Fz = current_Fz
        print("prev_Fz:", prev_Fz)
        counter_ += 1
    # print("force_diff",current_Fz-prev_Fz)
    if current_Fz - prev_Fz>=3:
        

        if counter == 0:
           t1 = rospy.get_time()
           print("time_1:", t1)

        t2 = rospy.get_time()
        diff_time = t2-t1
        if diff_time <= 60:
            print("THz is recording")
            print("current_Fz", current_Fz)
            # print("numpoints", numpoints)
            pulse_data = submit("GETLATESTPULSE", "d", 8*numpoints[0]) #array with the pulse data
            print("size",sys.getsizeof(pulse_data))
            print("counter",counter)
            # plt.plot(pulse_data)
            # plt.show()
        else:
            return_value = submit("STOP","?", 8)#Starting/Stopping Spectrometer if scanning
        
        counter += 1


if __name__=="__main__":
    rospy.init_node("terasmart_ros")
    rate = rospy.Rate(4)
    global prev_Fz, current_Fz, counter, counter_, t1, numpoints
    counter = 0
    counter_ = 0
    prev_Fz = 0
    current_Fz = 0
    # ip_addr = "137.205.241.135"
    ip_addr = "10.216.120.118"
    port = 8001
    start_time = -340
    end_time = -290
    return_value = submit("STOP","?", 8)#Starting/Stopping Spectrometer if scanning
    return_value = submit("GETSTATUS","i",32)#getting the status
    return_value = submit("SETSTARTVALUE -340", "?", 8)#setting the start of the scan
    return_value = submit("SETENDVALUE -290", "?", 8)#setting the end of the scan
    return_value = submit("START","?", 8)#StartingSpectrometer if scanning
    numpoints = submit("GETNUMBEROFPOINTS", "i", 32) #number of points of the pulse
    print("numpoints: ",  numpoints[0])
    
    time_axis = submit("GETTIMEAXIS", "d", 8*numpoints[0]) #array with the time data
    # plt.plot(time_axis)
    # plt.show()
    print("unfinished")

    while not rospy.is_shutdown():
        #rospy.wait_for_message("/cartesian_wrench")
        rospy.Subscriber("/cartesian_wrench_tool_ts",WrenchStamped,callback=CallbackWrench)
        rate.sleep()