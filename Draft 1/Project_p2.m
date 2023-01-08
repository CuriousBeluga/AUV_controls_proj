function [plotdata]=Project_p2()
clear all
close all
clc


%%XY plane


% Motion Control of Mobile Robots based on Linearization
% Example: Rectilinear
% Method #1 using ode45

clear all
close all
clc

Tf=15; % Total Time

actualx(1,:)=[0, 9.6, -140*pi/180]; % Initial Condition where the robot starts.
% format [x0, y0, z0 theta0 theta1]. Play around with the initital configuration and
% see the result.

 xdes_vec(1,:)=[30, 1, 2*pi]; % Initial Desired configuration
 E(1,:) = xdes_vec(1,:)-actualx(1,:); % Initial configuration error

x0=E(1,:); % Initial Input to the ode45 i.e. initial error

[T,X] = ode45(@(t,x)errordynamics(t,x),[0 Tf],x0); % ode to solve edot=A*e


%% 
function [dx ] = errordynamics(t,x)
        
    dr=1.8; % damping ration (PLAY AROUND WITH IT AND SEE THE RESULT)
    wd=0; % desired steering velocity (CALCULATING FROM TRAJECTORY INFO)
    vd=1.5; % desired linear velocity (CALCULATING FROM TRAJECTORY INFO)
    nf=1; % natural frequency (PLAY AROUND WITH IT AND SEE THE RESULT)
    k1=2*dr*nf; % controller gain
    k3=k1; % controller gain
    k2=(nf^2-wd^2)/vd; % controller gain
              
        dx=zeros(3,1);
        
        % Equation 11.68 in the siciliano's book can be written as follows
        % in the state space format
        dx(1) = -k1*x(1);
        dx(2) = 3*x(3);
        dx(3) = -k2*x(2)-k3*x(3);
end
    
%% 
% The output of the ode45 for edot=A*e is e! Therefore, e=X, so:
xerror=X(:,1); % error in x
yerror=X(:,2); % error in y
terror=X(:,3); % error in theta

error_vec=[xerror, yerror, terror]; % error vector
[T_r_2,tc] = size(T);
z_level(T_r_2,:)=zeros;

for i=1:size(T)
    % calculating the desired trajectory
    xdes=3+1*T(i); % COMING FROM THE (PLANNED) TRAJECTORY INFORMATION
    ydes=sin(3*T(i)); % COMING FROM THE TRAJECTORY INFORMATION 
    tdes=30*pi/180; % Theta_desired: COMING FROM THE TRAJECTORY INFORMATION
    xdes_vec(i,:)=[xdes, ydes, tdes]; % desired trajectory
    
    z_level(i,:) = 1;

end

actualx=xdes_vec - error_vec; % Actual configuration of the robot

plotdata(T_r_2,3) =zeros;
 
 for i=1:size(T)
 plotdata(i,:) = [actualx(i,1),actualx(i,2),z_level(i,1)];


 end


% Plot
figure('Name','Trajectory Plot')
plot(actualx(:,1), actualx(:,2),'b','LineWidth', 4);
hold on 
plot(xdes_vec(:,1), xdes_vec(:,2),'r','LineWidth', 4);

%%PLot 3D space
figure('Name','3D Trajectory')
plot3(actualx(:,1),actualx(:,2),z_level,'b','LineWidth', 4)


figure('Name','Cartesian Error')
hold on
plot(T,abs(X(:,1))+abs(X(:,2)),'r', 'LineWidth', 4);

end