function wrenchCallback(~, current_wrench_msg)
    global count initial_wrench_Fz pulse pulse_rec numpoints t1 current_wrench_Fz_rec current_wrench_sub
    current_wrench_Fz = (current_wrench_msg.Wrench.Force.Z);
    if (current_wrench_Fz - initial_wrench_Fz >= 3 )
         if count  == 1
            t1 = rostime("now");
%             t1_ = t1.Sec;
         end
          disp(current_wrench_Fz - initial_wrench_Fz)
          
         trigger = 1;
         t2 = rostime("now");
         diff_time = t2.Sec-t1.Sec

         if diff_time <= 60
            current_wrench_Fz_rec(count) = current_wrench_Fz;
            disp('TeraSmart is now Recording');
            
            pulse = submit("GETLATESTPULSE",'float64',numpoints);
            figure(1);
            plot(pulse')
            pulse_rec(count,:) = pulse; 
            count = count+1;
%             size(pulse_rec);
             pause(0.25)
         elseif diff_time > 60
             clear current_wrench_sub
         end

    else
        trigger = 0;
    end
    
end
