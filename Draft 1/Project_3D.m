function []=Project_3D()

p1_data = Project_p1;
p2_data = Project_p2;
cp = 56;
cp2 = 1;


%%PLot 3D space with drift factor
figure('Name','3D Trajectory')
plot3(p1_data(:,1), p1_data(:,2), p1_data(:,3),'b','LineWidth', 4)
hold on
plot3(p2_data(:,1)+cp, p2_data(:,2)+cp2, p2_data(:,3),'g','LineWidth', 4)

legend('Descent Phase','Scan Phase','Location','northeast')

end