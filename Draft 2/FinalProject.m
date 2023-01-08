close all
clc


Tf = 50;	% Total time for tracking the trajectory

dt=0.01; % time increment
tsteps=[0:dt:Tf]; % time steps
N=size(tsteps,2);

%% Initial Configuration of the robot
X(:,1)=[0, 2, 0, 0, 0, 0]; % x, y, z, theta, rudder, velocity
X_error(:,1) = [0,0,0];
X_v_over_t = [];

th_max_fwd = 1.5;
th_max_rev = -0.25;
max_rud_pos = deg2rad(60);
max_z = 1;

th_gain = 2;
z_gain = 4;
r_gain = 400;
kd_z = .0524;
kd_th = .025;
kd_r = .023;

fwd_offset = 0.25;

for i=1:N
    t=tsteps(i);

    %Robot state
    x_pos = X(1,i); %Desired state
    y_pos = X(2,i); %Desired state
    z_pos = X(3,i); %Desired state
    t_pos = X(4,i); %Free variable THETA in WORLD COORDS
    r_pos = X(5,i); %Controlled Variable THETA of RUDDER
    th_vel = X(6,i); %Controlled Variable THRUSTER VELOCITY
    
    %Optional current that messes with the robot
    XYZ_Current(:,i) = [0.25*sin(t/5);
                   0.25*sin(t/6);
                   0.25*sin(t/7)];
    
    %Desired trajectory
    x_eq = 3*cos(t/3)-3;
    y_eq = 3*sin(t/3)+3;
    z_eq = -0.7*t;
    if(z_eq < -10)
        z_eq = -10;
        x_eq = 5*cos(t/6)+2;
        y_eq = 5*sin(t/6)-3.5;
    end
    
    x_des = x_eq;
    y_des = y_eq;
    z_des = z_eq;
    Xdes(:,i)=[ x_des;
            y_des;
            z_des];

    %Position Errors
    XY_error = sqrt((x_des-x_pos)^2 + (y_des-y_pos)^2);
    Z_error = z_des-z_pos;
    
    %Rotation Error
    
%     t_error = t_pos - atan2(y_des-y_pos, x_des-x_pos);
    t_error1 = inv(T(t_pos,x_pos,y_pos))*[x_des;y_des;1];
    t_error = t_error1(2);
    
 
    %Control functions (PD control)
    
    if i==1
    th_vel = th_gain*XY_error;
    z_vel = z_gain*Z_error; 
    r_vel = r_gain*t_error;
    r_vel = min(max(r_vel,-10*max_rud_pos),10*max_rud_pos); %%saturation filter
    r_pos = r_vel*dt+r_pos;
    end

    if i >1

    th_vel = th_gain*XY_error-kd_th*X(1,i-1);
    z_vel = z_gain*Z_error-kd_z*X(2,i-1); 
    r_vel = r_gain*t_error-kd_r*X(3,i-1);
    r_vel = min(max(r_vel,-10*max_rud_pos),10*max_rud_pos); %%saturation filter
    r_pos = r_vel*dt+r_pos;
    
    end


    %Check overloading state conditions: (SATURATION)
    th_vel = min(max(th_vel,th_max_rev),th_max_fwd);
    r_pos = min(max(r_pos,-max_rud_pos),max_rud_pos);
    z_vel = min(max(z_vel,-max_z),max_z);

    %Bring in the next state of the robot! X,Y,Z, Rotation!
    nxt_x = th_vel*cos(t_pos)*dt + X(1,i); %times dt  Shadowing the trajectory
    nxt_y = th_vel*sin(t_pos)*dt + X(2,i); %times dt
    nxt_z = z_vel*dt + X(3,i); %cap at submergence rate, through control function.
    nxt_t = th_vel*r_pos*dt + X(4,i); %thruster velocity * - rudder position
    
%     X_velocity = [T(t_pos)*[y1_d;y2_d]];
%     X_v_over_t(:,i+1) = X_velocity;

% X(:,i+1)= [X_velocity(1)*cos(t_pos);X_velocity(1)*sin(t_pos);X_velocity(2)]*dt + X(:,i); % Descrete time method of solving differential equation

X(:,i+1) = [nxt_x;nxt_y;nxt_z;nxt_t;r_pos;th_vel];

X(1:3,i+1) = X(1:3,i+1) + XYZ_Current(i)*dt;

X_error(:,i+1) = [XY_error;Z_error;t_error;];

end

size(tsteps)
size(X)

%plot
figure
plot(tsteps, X(5:6,1:end-1), 'LineWidth', 2)
title('rudder angle and robot velocity');
legend('Rudder Angle','Robot Velocity')

figure
plot(tsteps, X_error(:,1:end-1), 'LineWidth', 2);
title('error plot');
legend('XY Error','Z Error','Theta Error')

figure
plot3(Xdes(1,:),Xdes(2,:),Xdes(3,:));
hold on
plot3(X(1,:),X(2,:),X(3,:));
title('Desired Trajectory vs. Modeled Trajectory')
legend('Desired Trajectory','Simulated Trajectory')

figure
plot(tsteps(1:end-1), XYZ_Current(:,1:end-1));
title('Forcing Currents');

function trans = T(theta,x,y)
trans = [cos(theta) -sin(theta) x;
        sin(theta) cos(theta) y;
        0 0 1];
end
