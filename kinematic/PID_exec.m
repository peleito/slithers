clear;
clc;
close all;

xk = 0;
theta1 = 0;
theta2 = 0;
x_des = 1;
theta1_des = pi/4;
theta2_des = pi/4;
%kp = 9000
Kc = 100;
Pc = 4.5; %sec

K_b = 5*[0.95*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for the base
K_t1 = 5*[1.5*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for theta 1
K_t2 = 5*[1.5*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for theta 2

xcur = xk;
theta1_cur = theta1;
theta2_cur = theta2;
pid = PID(xcur, theta1_cur, theta2_cur, x_des, theta1_des, theta2_des, K_b, K_t1, K_t2);

% for j= 1:5
%     y = [];
%     pid = PID(xcur, theta1_cur, theta2_cur, xcur + .004*j, theta1_cur + .004*j, theta2_cur + .004*j, K_b, K_t1, K_t2);
times = []
time = 0
x_plot = []
theta1_plot = []
theta2_plot = []
for k=1:200
             %y(k) = pid.x_k(2,1);
             %plot(1:k,y(1:k),'-'); % use the loop indices to plot in a loop
             pid = pid.update_x_k();
             pid = pid.compute_output();
             x_plot(k) = pid.x_k(1,1);
             theta1_plot(k) = pid.x_k(2,1);
             theta2_plot(k) = pid.x_k(3,1);
             pause(0.01)
             times(k) = time
             time = time + 0.01
end
setpoint_x = ones(1, length(times)) * x_des
setpoint_theta1 = ones(1, length(times)) * theta1_des
setpoint_theta2 = ones(1,length(times)) * theta2_des

%     hold on
% end
figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,3,1)
plot(times,x_plot, times, setpoint_x,  '--r', 'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$x$ (meters)','FontSize',40,'Interpreter','latex')
axis square

subplot(1,3,2)
plot(times,theta1_plot, times, setpoint_theta1, '--r', 'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_1$ (rad)','FontSize',40,'Interpreter','latex')
axis square

subplot(1,3,3)
plot(times,theta2_plot, times, setpoint_theta2, '--r', 'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_2$ (rad)','FontSize',40,'Interpreter','latex')
axis square

export_fig pid_val.png -transparent -native