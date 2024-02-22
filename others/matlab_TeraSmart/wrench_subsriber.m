clear all
clc
close all
global initial_wrench_Fz pulse pulse_rec numpoints count current_wrench_Fz_rec current_wrench_sub time
retvalue = submit("STOP",'uint8',1);
rosshutdown;
rosinit;
starttime=-340;
endtime=-280;
retvalue = submit("START",'uint8',1);


setvalue = submit("GETSTATUS",'int32',1);

starttimecommand = sprintf("SETSTARTVALUE %f",starttime);
retvalue = submit(starttimecommand,'uint8',1);

endtimecommand = sprintf("SETENDVALUE %f",endtime);
retvalue = submit(endtimecommand,'uint8',1);

setmodecommand = sprintf("SETMODE %d",0);
retvalue = submit(setmodecommand,'uint8',1);

numpoints = submit("GETNUMBEROFPOINTS",'int32',1);


time = submit("GETTIMEAXIS",'float64',numpoints);
time=time';
pulse=pulse';
count = 1;
Init_wrench_sub = rossubscriber('/cartesian_wrench_tool_ts','DataFormat', 'struct');
[initial_wrench_msg,status, statustext]  = receive(Init_wrench_sub,2.0);
initial_wrench_Fz = (initial_wrench_msg.Wrench.Force.Z)
current_wrench_sub = rossubscriber('/cartesian_wrench_tool_ts',@wrenchCallback, 'DataFormat', 'struct');
