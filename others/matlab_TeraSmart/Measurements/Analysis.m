Pulse{1} = table2array(readtable('Pulse_recorded_Anu_forearm1.txt'));
Pulse{2} = table2array(readtable('Pulse_recorded_Anu_forearm2.txt'));
Pulse{3} = table2array(readtable('Pulse_recorded_Anu_forearm3.txt'));
% Pulse{4} = table2array(readtable('Pulse_recorded_Anu_forearm4.txt'));
Pulse{4} = table2array(readtable('Pulse_recorded_Anu_forearm5.txt'));

Force{1} = table2array(readtable('Force_recorded_Anu_forearm1.txt'));
Force{2} = table2array(readtable('Force_recorded_Anu_forearm2.txt'));
Force{3} = table2array(readtable('Force_recorded_Anu_forearm3.txt'));
% Force{4} = table2array(readtable('Force_recorded_Anu_forearm4.txt'));
Force{4} = table2array(readtable('Force_recorded_Anu_forearm5.txt'));
%%
for i=1:4
    Pulse_rec = Pulse{1,i}(:,2:end)';
    P2P=max(Pulse_rec(:,end/2:end),[],2)-min(Pulse_rec(:,end/2:end),[],2);
    figure(1),plot(P2P)
     hold on
%     figure(2),plot(current_wrench_Fz_rec)
end

%%
for i=1:4
    force_rec = Force{1,i};
    for j = 1:size(force_rec,1)
        if force_rec(j) == 0
            force_rec(j) = force_rec(j-1);
        end
    end
    figure(2),plot(force_rec)
    hold on
end
