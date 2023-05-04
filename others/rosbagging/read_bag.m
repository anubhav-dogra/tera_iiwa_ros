bag = rosbag('2023-05-04-15-04-21.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
 Fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
 figure(1), plot(Fz, 'LineWidth',1.5);
 grid on
 fontsize(gcf,16,"points")

 bag = rosbag('2023-05-04-15-09-54.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
 Fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
 figure(2), plot(Fz, 'LineWidth',1.5);
 grid on
 fontsize(gcf,16,"points")

 bag = rosbag('2023-05-04-15-17-45.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
 Fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
 figure(3), plot(Fz, 'LineWidth',1.5);
 grid on
 fontsize(gcf,16,"points")

 bag = rosbag('2023-05-04-15-21-33.bag');
bSel = select(bag,'Topic','/cartesian_wrench_tool');
msgStructs = readMessages(bSel,'DataFormat','struct');
 Fz = cellfun(@(m) double(m.Wrench.Force.Z),msgStructs);
 figure(4), plot(Fz, 'LineWidth',1.5);
 grid on
 fontsize(gcf,16,"points")
