clear all
clc
close all
global initial_wrench_Fz trigger pulse pulse_rec numpoints count
rosshutdown;
rosinit;
starttime=-340;
endtime=-290;
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
Init_wrench_sub = rossubscriber('/cartesian_wrench','DataFormat', 'struct');
[initial_wrench_msg,status, statustext]  = receive(Init_wrench_sub,2.0);
initial_wrench_Fz = (initial_wrench_msg.Force.Z)
trigger = 0;
 current_wrench_sub = rossubscriber('/cartesian_wrench',@wrenchCallback, 'DataFormat', 'struct');
%%
size(pulse_rec)
close all

P2P=max(pulse_rec(:,end/2:end),[],2)-min(pulse_rec(:,end/2:end),[],2);
figure(1),plot(P2P)
