clear;
rosshutdown;
rosinit;

global initial_wrench_Fz trigger
Init_wrench_sub = rossubscriber('/cartesian_wrench','DataFormat', 'struct');
[initial_wrench_msg,status, statustext]  = receive(Init_wrench_sub,2.0);
initial_wrench_Fz = abs(initial_wrench_msg.Force.Z)
trigger = 0;
current_wrench_sub = rossubscriber('/cartesian_wrench',@wrenchCallback, 'DataFormat', 'struct');


function wrenchCallback(~, current_wrench_msg)
    global initial_wrench_Fz current_wrench_Fz trigger
    current_wrench_Fz = abs(current_wrench_msg.Force.Z);
    
    if (current_wrench_Fz - initial_wrench_Fz >= 3 )
         disp(current_wrench_Fz - initial_wrench_Fz)
        trigger = 1;
        disp('TeraSmart is now Recording')

    else
        trigger = 0;
    end
end

