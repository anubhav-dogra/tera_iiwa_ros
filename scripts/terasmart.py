import socket, struct

def submit(command, type_id, n_bit):
    sock = socket.socket()
    sock.connect((ip_addr,port))
    sock.sendall(command.encode('utf-8'))
    return_value = [e[0] for e in list(struct.iter_unpack(type_id, sock.recv(n_bit)))]
    sock.close()
    return return_value



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
# print("unfinished")
return_value = submit("STOP","?", 8)#Starting/Stopping Spectrometer if scanning