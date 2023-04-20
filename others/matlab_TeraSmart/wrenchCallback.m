function wrenchCallback(~, current_wrench_msg)
    global count initial_wrench_Fz current_wrench_Fz trigger pulse pulse_rec numpoints t1
    current_wrench_Fz = (current_wrench_msg.Force.Z)
    if (current_wrench_Fz - initial_wrench_Fz >= 3 )
         if count  == 1
            t1 = rostime("now");
         end
          disp(current_wrench_Fz - initial_wrench_Fz)
         trigger = 1;
         t2 = rostime("now");
         diff_time = t2.Sec-t1.Sec

         if diff_time <= 60
            disp('TeraSmart is now Recording')
            pulse = submit("GETLATESTPULSE",'float64',numpoints);
            figure(1);
            plot(pulse')
            pulse_rec(count,:) = pulse; 
            count = count+1;
            size(pulse_rec);
            pause(0.25);
         else
         end

    else
        trigger = 0;
    end
end
