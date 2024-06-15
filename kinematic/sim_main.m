%% Simulator for a moveable two-link system
clear
close all
clc

%% Parameters definition
global L1 L2 base_size stateMin stateMax
L1 = 1.2; % m
L2 = 1.2; % m
base_size = 0.3; % m
M1 = 10; % kg
M2 = 10; % kg
Mb = 30; % kg

stateMin = [-2.5 -pi -pi];
stateMax = [2.5 pi pi];

mlb = M1+M2+Mb; % kg
Il1 = 1/3*M1*L1^2 + M2*L1^2; % kg m^2
Il2 = 1/3*M2*L2^2; % kg m^2
dt = 0.1; % sec

A = [1 0 0 dt 0 0;
     0 1 0 0 dt 0;
     0 0 1 0 0 dt;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

B = [0 0 0;
     0 0 0;
     0 0 0;
     dt/mlb 0 0;
     0 dt/Il1 0;
     0 0 dt/Il2];

%% Pick the path to test here (uncomment the desired path)
%% crescent
% str = 'cresent';
% t = linspace(-pi/2,pi/2,100);
% x_desired1 = 4*cos(t+pi)+2;
% y_desired1 = 0.25*sin(t+pi)+1.7;
% x_desired2 = 2*cos(t+pi)+2;
% y_desired2 = 0.25*sin(t+pi)+1.7;
% x_desired = [x_desired1 flip(x_desired2)];
% y_desired = [y_desired1 flip(y_desired2)];

% ee_desired = [x_desired;
%               y_desired];
% figure('units','normalized','outerposition',[0 0 1 1])
% subplot(1,3,3)
% plot(ee_desired(1,:),ee_desired(2,:),'LineWidth',2)
% set(gca,'fontsize',24)
% xlabel('x (meters)','FontSize',40,'Interpreter','latex')
% ylabel('y (meters)','FontSize',40,'Interpreter','latex')
% axis square
% title('Crescent','FontSize',48,'Interpreter','latex')

%% heart
% str = 'heart';
% t = linspace(-pi,pi,100);
% x_desired = sin(t).^3;
% y_desired = (13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))/32+1.5;

% ee_desired = [x_desired;
%               y_desired];
% subplot(1,3,2)
% plot(ee_desired(1,:),ee_desired(2,:),'LineWidth',2)
% set(gca,'fontsize',24)
% xlabel('x (meters)','FontSize',40,'Interpreter','latex')
% ylabel('y (meters)','FontSize',40,'Interpreter','latex')
% axis square
% title('Heart','FontSize',48,'Interpreter','latex')

%% sine wave
str = 'sine';
x_desired = linspace(-2,2,100);
y_desired = 1/4*sin(8*x_desired)+2;

ee_desired = [x_desired;
              y_desired];
% subplot(1,3,1)
% plot(ee_desired(1,:),ee_desired(2,:),'LineWidth',2)
% set(gca,'fontsize',24)
% xlabel('x (meters)','FontSize',40,'Interpreter','latex')
% ylabel('y (meters)','FontSize',40,'Interpreter','latex')
% axis square
% title('Sine Wave','FontSize',48,'Interpreter','latex')

% export_fig test_paths.png -transparent -native

%% Initialize
figure
init_x = -1;
init_theta1 = pi/2;
init_theta2 = -pi/3;

step_size = length(x_desired);
end_effector_pose = zeros(2,step_size);

x = zeros(1,step_size);
theta1 = zeros(1,step_size);
theta2 = zeros(1,step_size);

x(1) = init_x;
theta1(1) = init_theta1;
theta2(1) = init_theta2;

% x = linspace(-1,1,step_size);
% theta1 = linspace(pi/2,pi,step_size);
% theta2 = linspace(-pi/3,pi/3,step_size);

%% Plot
plot_handles = create_base_link_sim_plot(L1, L2, base_size, init_x, init_theta1, init_theta2, end_effector_pose, ee_desired);

for plot_step = 1:step_size
    tic
    [x_d, theta1_d, theta2_d] = inverse_kinematics(x, theta1, theta2, plot_step, ee_desired, dt);
    timer(plot_step) = toc;
    x(plot_step) = x_d(1);
    theta1(plot_step) = theta1_d(1);
    theta2(plot_step) = theta2_d(1);
    
    end_effector_pose = update_base_link_sim_plot(plot_handles, x(plot_step), theta1(plot_step), theta2(plot_step), plot_step, end_effector_pose, ee_desired);
    pause(0.1)
end

pause(1)
%% Plot desired vs actual
figure
plot(ee_desired(1,:),ee_desired(2,:),end_effector_pose(1,:),end_effector_pose(2,:))
pause(1)

%% RMSE
residuals = ee_desired-end_effector_pose;
rmse = sqrt(mean(residuals.^2,"all"));
mean_time = mean(timer);

%% Remove first point
x = x(2:end);
theta1 = theta1(2:end);
theta2 = theta2(2:end);

%% Velocity
time = linspace(0,step_size*dt,step_size);

dx = diff(x);
dtheta1 = diff(theta1);
dtheta2 = diff(theta2);

dxdt = dx/dt;
dtheta1dt = dtheta1/dt;
dtheta2dt = dtheta2/dt;

max_vel = [max(abs(dxdt)) max([abs(dtheta1dt) abs(dtheta2dt)])];

%% Acceleration
dx2 = diff(dxdt);
dtheta12 = diff(dtheta1dt);
dtheta22 = diff(dtheta2dt);

dx2dt2 = dx2/dt;
dtheta12dt2 = dtheta12/dt;
dtheta22dt2 = dtheta22/dt;

max_acc = [max(abs(dx2dt2)) max([abs(dtheta12dt2) abs(dtheta22dt2)])];

%% Jerk
dx3 = diff(dx2dt2);
dtheta13 = diff(dtheta12dt2);
dtheta23 = diff(dtheta22dt2);

dx3dt3 = dx3/dt;
dtheta13dt3 = dtheta13/dt;
dtheta23dt3 = dtheta23/dt;

max_jer = [max(abs(dx3dt3)) max([abs(dtheta13dt3) abs(dtheta23dt3)])];

%% Metrics
str
rmse
mean_time
max_vel
max_acc
max_jer

%% Plots

figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,3,1)
plot(time(1:end-2),dxdt,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\dot{x}$ (meters/sec)','FontSize',40,'Interpreter','latex')
axis square
title('Base','FontSize',48,'Interpreter','latex')

subplot(1,3,2)
plot(time(1:end-2),dtheta1dt,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\dot{\theta_1}$ (rad/sec)','FontSize',40,'Interpreter','latex')
axis square
title('Joint 1','FontSize',48,'Interpreter','latex')

subplot(1,3,3)
plot(time(1:end-2),dtheta2dt,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\dot{\theta_2}$ (rad/sec)','FontSize',40,'Interpreter','latex')
axis square
title('Joint 2','FontSize',48,'Interpreter','latex')
% export_fig state_vel.png -transparent -native