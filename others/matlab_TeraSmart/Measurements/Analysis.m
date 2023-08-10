% Pulse{1} = table2array(readtable('Pulse_recorded_Anu_forearm1.txt'));
% Pulse{2} = table2array(readtable('Pulse_recorded_Anu_forearm2.txt'));
% Pulse{3} = table2array(readtable('Pulse_recorded_Anu_forearm3.txt'));
% % Pulse{4} = table2array(readtable('Pulse_recorded_Anu_forearm4.txt'));
% Pulse{4} = table2array(readtable('Pulse_recorded_Anu_forearm5.txt'));

% Force{1} = table2array(readtable('Force_recorded_Anu_forearm1.txt'));
% Force{2} = table2array(readtable('Force_recorded_Anu_forearm2.txt'));
% Force{3} = table2array(readtable('Force_recorded_Anu_forearm3.txt'));
% Force{4} = table2array(readtable('Force_recorded_Anu_forearm4.txt'));
% Force{4} = table2array(readtable('Force_recorded_Anu_forearm5.txt'));
% Force{1} = table2array(readtable('Force_study_270423/Force_recorded_Shruti_Rforearm1_270423.txt'));
% Force{2} = table2array(readtable('Force_study_270423/Force_recorded_Shruti_Rforearm2_270423.txt'));
% Force{3} = table2array(readtable('Force_study_270423/Force_recorded_Shruti_Rforearm3_270423.txt'));
% Force{4} = table2array(readtable('Force_study_270423/Force_recorded_Shruti_Rforearm4_270423.txt'));
% Pulse{1} = table2array(readtable('no_touch/arturo_force_aug_3/Pulse_recorded1.txt'));
% Pulse{2} = table2array(readtable('no_touch/arturo_force_aug_3/Pulse_recorded2.txt'));
% Pulse{3} = table2array(readtable('no_touch/arturo_force_aug_3/Pulse_recorded3.txt'));
% Pulse{4} = table2array(readtable('no_touch/arturo_force_aug_3/Pulse_recorded4.txt'));
% Force{1} = table2array(readtable('no_touch/arturo_force_aug_3/Force_recorded1.txt'));
% Force{2} = table2array(readtable('no_touch/arturo_force_aug_3/Force_recorded2.txt'));
% Force{3} = table2array(readtable('no_touch/arturo_force_aug_3/Force_recorded3.txt'));
% Force{4} = table2array(readtable('no_touch/arturo_force_aug_3/Force_recorded4.txt'));
%% Shruti Face
Pulse{1} = table2array(readtable('aug_4/Pulse_recorded_shruti_cheek_1.txt'));
Pulse{2} = table2array(readtable('aug_4/Pulse_recorded_shruti_cheek_2.txt'));
Pulse{3} = table2array(readtable('aug_4/Pulse_recorded_shruti_cheek_3.txt'));
% Pulse{4} = table2array(readtable('aug_4/Pulse_recorded_shruti_forehead_4.txt'));
Force{1} = table2array(readtable('aug_4/Force_recorded_shruti_cheek_1.txt'));
Force{2} = table2array(readtable('aug_4/Force_recorded_shruti_cheek_2.txt'));
Force{3} = table2array(readtable('aug_4/Force_recorded_shruti_cheek_3.txt'));
% Force{4} = table2array(readtable('aug_4/Force_recorded_shruti_forehead_4.txt'));
%%
for i=1:3
    Pulse_rec = Pulse{1,i}(:,2:end)';
    P2P=max(Pulse_rec(:,end/2:end),[],2)-min(Pulse_rec(:,end/2:end),[],2);
    figure(1),plot(P2P,'LineWidth',2.0)
     hold on
        grid on,   grid on, fontsize(gcf,18,"points")
%     figure(2),plot(current_wrench_Fz_rec)
end

%%
% Init_force = [0.2757, -0.2192, 0.3997,-1.149];
for i=1:3
    force_rec1 = Force{1,i};
    for j = 1:size(force_rec1,1)
        if force_rec1(j) == 0
            force_rec1(j) = force_rec1(j-1);
        end
    end
    force_rec1 = force_rec1 - force_rec1(1);
%      force_rec = force_rec - Init_force(i);
    figure(3),plot(force_rec1, 'LineWidth',2.0)
    grid on, fontsize(gcf,18,"points")
  
 
    hold on
end
