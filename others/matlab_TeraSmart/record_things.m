size(pulse_rec)
size(current_wrench_Fz_rec)
close all
np = size(pulse_rec,1);
P2P=max(pulse_rec(:,end/2:end),[],2)-min(pulse_rec(:,end/2:end),[],2);
figure(1),plot(P2P)
figure(2),plot(current_wrench_Fz_rec)
T = table(time', pulse_rec', 'VariableNames', {'Time','Pulse'});
writetable(T, 'Measurements/Force_study/Pulse_recorded_Shruti_Rforearm4_270423.txt')
T1 = table(current_wrench_Fz_rec', 'VariableNames', {'Force'});
writetable(T1, 'Measurements/Force_study/Force_recorded_Shruti_Rforearm4_270423.txt')