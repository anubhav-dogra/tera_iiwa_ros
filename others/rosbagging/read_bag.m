
bag = rosbag('force_on_arm_400k.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
Fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
n = size(Fz,1);
time_init = double(msgStructs{1}.Header.Stamp.Sec);
time_final = double(msgStructs{n}.Header.Stamp.Sec);
n_sec = time_final-time_init;
time=linspace(0,n_sec,n);

figure('Color','w','units','normalized','OuterPosition',[.1 .2 .5 .5])
figure(1),plot(time, Fz-Fz(1), 'LineWidth',2);
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
