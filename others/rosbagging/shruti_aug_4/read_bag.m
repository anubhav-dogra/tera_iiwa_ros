
clc
clear all

bag = rosbag('cheeks.bag');
bSel = select(bag,'Topic','/tf_array_out');
msgStructs = readMessages(bSel,'DataFormat','struct');
w = cellfun(@(m) double(m.Poses.Orientation.W),msgStructs);
x = cellfun(@(m) double(m.Poses.Orientation.X),msgStructs);
y = cellfun(@(m) double(m.Poses.Orientation.Y),msgStructs);
z = cellfun(@(m) double(m.Poses.Orientation.Z),msgStructs);
xp = cellfun(@(m) double(m.Poses.Position.X),msgStructs);
yp = cellfun(@(m) double(m.Poses.Position.Y),msgStructs);
zp = cellfun(@(m) double(m.Poses.Position.Z),msgStructs);
n = size(x,1);
time_init = double(msgStructs{1}.Header.Stamp.Sec);
time_final = double(msgStructs{n}.Header.Stamp.Sec);
n_sec = time_final-time_init;
time=linspace(0,n_sec,n);
eul = (180/pi)*quat2eul([x,y,z,w],'XYZ');

figure('Color','w','units','normalized','OuterPosition',[.1 .2 .5 .5])
% figure(1),plot(time, w, 'LineWidth',2);
% hold on
% figure(1),plot(time, x, 'LineWidth',2);
% figure(1),plot(time, y, 'LineWidth',2);
% figure(1),plot(time, z, 'LineWidth',2);
% figure(1),plot(time, xp, 'LineWidth',2);
% figure(1),plot(time, yp, 'LineWidth',2);
% figure(1),plot(time, zp, 'LineWidth',2);
figure(2),plot(time, eul(:,1), 'LineWidth',2, 'Color','r');
hold on
figure(2),plot(time, eul(:,2), 'LineWidth',2, 'Color','g');
figure(2),plot(time, eul(:,3), 'LineWidth',2, 'Color','b');

% colormap1=jet(11);
% for i=1:11
% figure(1),plot(x,y+i,'color',colormap1(i,:),'LineWidth',1),hold on
% end
set(gca,'LineWidth',0.75,'FontSize',16,'XMinorTick','on','YMinorTick','on','TickLength',[.01 0.1], 'XMinorGrid','on','YMinorGrid','on');
xlabel('Time (sec)')
ylabel('Force (N)')
grid on

% tightfig;
%text(4,-.4,'Newtons','FontSize',16,'Color','b')
