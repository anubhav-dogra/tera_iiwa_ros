% close all
% clc
% clear
% rosshutdown
%%
starttime=-340;
endtime=-290;
% rosinit;
% rostopic list;
%% 
% retvalue = submit("STOP",'uint8',1);


setvalue = submit("GETSTATUS",'int32',1);

starttimecommand = sprintf("SETSTARTVALUE %f",starttime);
retvalue = submit(starttimecommand,'uint8',1);

endtimecommand = sprintf("SETENDVALUE %f",endtime);
retvalue = submit(endtimecommand,'uint8',1);

setmodecommand = sprintf("SETMODE %d",0);
retvalue = submit(setmodecommand,'uint8',1);

numpoints = submit("GETNUMBEROFPOINTS",'int32',1);

pulse = submit("GETLATESTPULSE",'float64',numpoints);
time = submit("GETTIMEAXIS",'float64',numpoints);
time=time';
pulse=pulse';
%Stop
%pause(.5);
%retvalue = submit("STOP",'uint8',1);