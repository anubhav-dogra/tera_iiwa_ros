close all
clc
clear
rosshutdown
%%
starttime=-340;
endtime=-290;
rosinit;
rostopic list;
%% 
%retvalue = submit("START",'uint8',1);


return
retvalue = submit("GETSTATUS",'int32',1);

starttimecommand = sprintf("SETSTARTVALUE %f",starttime);
retvalue = submit(starttimecommand,'uint8',1);

endtimecommand = sprintf("SETENDVALUE %f",endtime);
retvalue = submit(endtimecommand,'uint8',1);

setmodecommand = sprintf("SETMODE %d",0);
retvalue = submit(setmodecommand,'uint8',1);

numpoints = submit("GETNUMBEROFPOINTS",'int32',1);
message_recievd = 10;

for i=1:40
pulse = submit("GETLATESTPULSE",'float64',numpoints);
time = submit("GETTIMEAXIS",'float64',numpoints);
time=time';
pulse=pulse';
PULSE
figure(1),plot(time,pulse,'LineWidth',2)
pause(.1)
clc
end
%Stop
%pause(.5);
%retvalue = submit("STOP",'uint8',1);