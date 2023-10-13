%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

%% Screw joints
Lbx = 0.34;
% Lbx = 0;
Lby = 0.0;
Lbz = 0.384;
% Lbz = 0;
husky_base_to_ur_base = [Lbx,Lby,Lbz];
husky_ur = eye(4);
husky_ur(1:3,4) = husky_base_to_ur_base;
 
S1_w = [0,0,1]; % base angular
S1_v = -cross(S1_w,[0,0,0]);
S1 = [S1_w,S1_v];

S2_w = [0,0,0]; % base linear
S2_v = [1,0,0];
S2 = [S2_w,S2_v];

S3_w = [0,0,1]; % shoulder pan
S3_v = -cross(S3_w,[0,0,0.163]+husky_base_to_ur_base);
S3 = [S3_w,S3_v];

S4_w = [0,1,0]; % shoulder lift
S4_v = -cross(S4_w,[0,0,0.163]+husky_base_to_ur_base);
S4 = [S4_w,S4_v];

S5_w = [0,1,0]; % elbow
S5_v = -cross(S5_w,[0.425,0,0.162]+husky_base_to_ur_base);
S5 = [S5_w,S5_v];

S6_w = [0,1,0]; % wrist 1
S6_v = -cross(S6_w,[0.817,0.133,0.162]+husky_base_to_ur_base);
S6 = [S6_w,S6_v];

S7_w = [0,0,-1]; % wrist 2
S7_v = -cross(S7_w,[0.817,0.133,0.063]+husky_base_to_ur_base); 
S7 = [S7_w,S7_v];

S8_w = [0,1,0]; % wrist 3
S8_v = -cross(S8_w,[0.817,0.233,0.063]+husky_base_to_ur_base);
S8 = [S8_w,S8_v];

parameters.screws = [S1; % base angular velocity
                     S2; % base linear velocity
                     S3; % shoulder pan
                     S4; % shoulder lift
                     S5; % elbow
                     S6]; % wrist 1
%                      S7; % wrist 2
%                      S8]; % wrist 3 screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

%% Parameters definition
parameters.config_state = [-1,0,0,0.817+Lbx;
                           0,0,1,0.233+Lby;
                           0,1,0,0.063+Lbz;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
% parameters.adjoint = eye(6); % adjoint matrix for mobile manipulator (se3 matrix, current pose)
parameters.stateMin = [-2*pi,-5,-2*pi,-2*pi,-2*pi,-2*pi];%,-2*pi,-2*pi]';
parameters.stateMax = [2*pi,5,2*pi,2*pi,2*pi,2*pi];%,2*pi,2*pi]';
parameters.dt = 0.2;
parameters.lambda_e = [100,100,100,100,100,100]';
parameters.lambda_j = 0.000001;
% parameters.lambda_v = [10,10,5,5,5,1,1,1]'/; % length of n
parameters.lambda_v = 10*[0.05,0.05,0.025,0.025,0.01,0.01];%,0,0]'; % length of n
parameters.time = 20;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = 2;
parameters.arm_dof = 4;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;
dof.base = parameters.base_dof;
dof.arm = parameters.arm_dof;
dof.total = parameters.total_dof;
dof2.base = 2;
dof2.arm = 0;
dof2.total = 2;


%% Initialize
% set array of pose goals (4 by 4 by k)
pose_goals_helix = generate_helix(2.5,0.0125,parameters.dt,parameters.time); %% working
pose_goals_sine = generate_sine(0.25,0.25,parameters.dt,parameters.time); %% working
pose_goals_horizontal = generate_horizontal_helix(0.5,1,parameters.dt,parameters.time); %% working

timer = zeros([parameters.steps,3]);
x = zeros([6,1]);
xdot = zeros([6,1,3]);
dx = zeros([6,1]);
parameters.base_pose = eye(4);
tool_pose = zeros([4,4,parameters.steps,3]);
error = zeros([4,4,parameters.steps,3]);


%% Main
pose = eye(4);

for trial = 1:1:3
    if trial == 1
        pose_goals = pose_goals_helix;
    elseif trial == 2
        pose_goals = pose_goals_sine;
    else
        pose_goals = pose_goals_horizontal;
    end

%     timer = zeros([1,parameters.steps]);
    x = zeros([6,1]);
%     xdot = zeros([6,1,3]);
    dx = zeros([6,1]);
    parameters.base_pose = eye(4);

    for plot_step = 1:parameters.steps
    
        % get next pose goal
        ee_desired = tform(pose_goals(plot_step));
    
        parameters.base_pose = parameters.base_pose*step_forward(parameters.screws,dx,dof2,parameters.dt);
        parameters.base_pose;
    
        % perform ik optimization
        tic
        dx = ik_optimization(x, xdot, ee_desired, parameters);
        xdot(:,plot_step,trial) = dx';
        timed = toc;
        timer(plot_step,trial) = timed;
        
        tool_pose(:,:,plot_step,trial) = parameters.base_pose*step_forward(parameters.screws,dx,dof,parameters.dt)*parameters.config_state;
        tool_pose(:,:,plot_step,trial);
        x(1:2) = [0;0];
        x(3:end) = dx(3:end);
    
    %     goal = manifold_to_vector(ee_desired);
        goal = ee_desired;
    %     actual = manifold_to_vector(tform(tool_pose(plot_step)));
        actual = tool_pose(:,:,plot_step,trial);
        error(:,:,plot_step,trial) = goal-actual;
        error(:,:,plot_step,trial);
    
    end

end
%% Plot parameters
linewidth = 2;
fontsize = 20;
labelsize = 28;
titlesize = 40;
save = false;
time = seconds(0:parameters.dt:parameters.time-parameters.dt);

%% Plot paths
figure('units','normalized','outerposition',[0 0 1 1])
subplot(1,3,1)
plotTransforms(pose_goals_helix)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Vertical Helix','FontSize',titlesize/2,'Interpreter','latex')

subplot(1,3,2)
plotTransforms(pose_goals_sine)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Sine Wave','FontSize',titlesize/2,'Interpreter','latex')

subplot(1,3,3)
plotTransforms(pose_goals_horizontal)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Horizontal Helix','FontSize',titlesize/2,'Interpreter','latex')

if save
    pause(5)
    export_fig test_paths.png -transparent -native
    pause(5)
    close
end

%% Plot error

figure('units','normalized','outerposition',[0 0 1 1])

% vertical helix
pose_goal = tform(pose_goals_helix);
error_plot.x = squeeze(tool_pose(1,4,:,1)-pose_goal(1,4,1:end-1));
error_plot.y = squeeze(tool_pose(2,4,:,1)-pose_goal(2,4,1:end-1));
error_plot.z = squeeze(tool_pose(3,4,:,1)-pose_goal(3,4,1:end-1));
for m = 1:1:length(tool_pose)
    vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,1));
    vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
    error_plot.rx(m) = vec1(1)-vec2(1);
    error_plot.ry(m) = vec1(2)-vec2(2);
    error_plot.rz(m) = vec1(3)-vec2(3);
end

error_plot.xtable = smoothdata(timetable(time',error_plot.x));
error_plot.ytable = smoothdata(timetable(time',error_plot.y));
error_plot.ztable = smoothdata(timetable(time',error_plot.z));
error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));

residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
rmse_pos(1) = sqrt(mean(residuals.^2,'all'));
residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
rmse_rot(1) = sqrt(mean(residuals.^2,'all'));

subplot(2,3,1)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
legend('x','y','z')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')


subplot(2,3,4)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
legend('roll','pitch','yaw')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square


% sine wave
pose_goal = tform(pose_goals_sine);
error_plot.x = squeeze(tool_pose(1,4,:,2)-pose_goal(1,4,1:end-1));
error_plot.y = squeeze(tool_pose(2,4,:,2)-pose_goal(2,4,1:end-1));
error_plot.z = squeeze(tool_pose(3,4,:,2)-pose_goal(3,4,1:end-1));
for m = 1:1:length(tool_pose)
    vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,2));
    vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
    error_plot.rx(m) = vec1(1)-vec2(1);
    error_plot.ry(m) = vec1(2)-vec2(2);
    error_plot.rz(m) = vec1(3)-vec2(3);
end

error_plot.xtable = smoothdata(timetable(time',error_plot.x));
error_plot.ytable = smoothdata(timetable(time',error_plot.y));
error_plot.ztable = smoothdata(timetable(time',error_plot.z));
error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));

residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
rmse_pos(2) = sqrt(mean(residuals.^2,'all'));
residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
rmse_rot(2) = sqrt(mean(residuals.^2,'all'));

subplot(2,3,2)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
legend('x','y','z')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Sine Wave','FontSize',titlesize,'Interpreter','latex')

subplot(2,3,5)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
legend('roll','pitch','yaw')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% horizontal helix
pose_goal = tform(pose_goals_horizontal);
error_plot.x = squeeze(tool_pose(1,4,:,3)-pose_goal(1,4,1:end-1));
error_plot.y = squeeze(tool_pose(2,4,:,3)-pose_goal(2,4,1:end-1));
error_plot.z = squeeze(tool_pose(3,4,:,3)-pose_goal(3,4,1:end-1));
for m = 1:1:length(tool_pose)
    vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,3));
    vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
    error_plot.rx(m) = vec1(1)-vec2(1);
    error_plot.ry(m) = vec1(2)-vec2(2);
    error_plot.rz(m) = vec1(3)-vec2(3);
end

error_plot.xtable = smoothdata(timetable(time',error_plot.x));
error_plot.ytable = smoothdata(timetable(time',error_plot.y));
error_plot.ztable = smoothdata(timetable(time',error_plot.z));
error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));

residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
rmse_pos(3) = sqrt(mean(residuals.^2,'all'));
residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
rmse_rot(3) = sqrt(mean(residuals.^2,'all'));

subplot(2,3,3)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
legend('x','y','z')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')

subplot(2,3,6)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
legend('roll','pitch','yaw')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

if save
    pause(5)
    export_fig error.png -transparent -native
    pause(5)
    close
end

%% Plot base linear states

figure('units','normalized','outerposition',[0 0 1 1])

states.base.x = squeeze(xdot(2,:,:));
states.base.xtable = smoothdata(timetable(time',states.base.x));
states.base.xdot = [zeros([1,3]);diff(states.base.xtable.Var1,1,1)];

states.base.xdottable = smoothdata(timetable(time',states.base.xdot/parameters.dt));

% linear x dot
subplot(1,2,1)
hold on
plot(states.base.xtable.Time,states.base.xtable.Var1(:,1),'LineWidth',linewidth)
plot(states.base.xtable.Time,states.base.xtable.Var1(:,2),'LineWidth',linewidth)
plot(states.base.xtable.Time,states.base.xtable.Var1(:,3),'LineWidth',linewidth)
legend('Vertical Helix','Sine Wave','Horizontal Helix')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{x}}$ (m/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Linear Velocity','FontSize',titlesize,'Interpreter','latex')

% linear x double dot
subplot(1,2,2)
hold on
plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,1),'LineWidth',linewidth)
plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,2),'LineWidth',linewidth)
plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,3),'LineWidth',linewidth)
legend('Vertical Helix','Sine Wave','Horizontal Helix')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\ddot{x}}$ (m/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Linear Acceleration','FontSize',titlesize,'Interpreter','latex')

if save
    pause(5)
    export_fig base_linear.png -transparent -native
    pause(5)
    close
end

%% Plot base angular states

figure('units','normalized','outerposition',[0 0 1 1])

states.base.omega = squeeze(xdot(1,:,:));
states.base.omegatable = smoothdata(timetable(time',states.base.omega));
states.base.omegadot = [zeros([1,3]);diff(states.base.omegatable.Var1,1,1)];
states.base.omegadottable = smoothdata(timetable(time',states.base.omegadot/parameters.dt));

% angular omega dot
subplot(1,2,1)
hold on
plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,1),'LineWidth',linewidth)
plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,2),'LineWidth',linewidth)
plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,3),'LineWidth',linewidth)
legend('Vertical Helix','Sine Wave','Horizontal Helix')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\omega}$ (rad/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Angular Velocity','FontSize',titlesize,'Interpreter','latex')

% angular omega double dot
subplot(1,2,2)
hold on
plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,1),'LineWidth',linewidth)
plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,2),'LineWidth',linewidth)
plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,3),'LineWidth',linewidth)
legend('Vertical Helix','Sine Wave','Horizontal Helix')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\omega}}$ (rad/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Angular Acceleration','FontSize',titlesize,'Interpreter','latex')

if save
    pause(5)
    export_fig base_angular.png -transparent -native
    pause(5)
    close
end

%% Plot joint states
figure('units','normalized','outerposition',[0 0 1 1])

states.arm.q = xdot(3:end,:,:);
states.arm.qtable = smoothdata(timetable(time',pagetranspose(states.arm.q)));
states.arm.qdot = [zeros([1,4,3]);diff(states.arm.qtable.Var1,1,1)];
states.arm.qdottable = smoothdata(timetable(time',(states.arm.qdot/parameters.dt)));

% vertical helix
subplot(2,3,1)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')

subplot(2,3,4)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% sine wave
subplot(2,3,2)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,2)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Sine Wave','FontSize',titlesize,'Interpreter','latex')

subplot(2,3,5)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,2)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% horizontal helix
subplot(2,3,3)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,3)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')

subplot(2,3,6)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,3)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

if save
    pause(5)
    export_fig arm_states.png -transparent -native
    pause(5)
    close
end

%% Table parameters
parameters.dt
parameters.lambda_e
parameters.lambda_v
parameters.lambda_j

%% Table metrics
comp_time = mean(timer);
max_lin_vel = max(states.base.xtable.Var1);
max_ang_vel = max(states.base.omegatable.Var1);
max_lin_accel = max(states.base.xdottable.Var1);
max_ang_accel = max(states.base.omegadottable.Var1);
states.base.xddot = [zeros([1,3]);diff(states.base.xdottable.Var1,1,1)];
states.base.xddottable = smoothdata(timetable(time',states.base.xddot/parameters.dt));
states.base.omegaddot = [zeros([1,3]);diff(states.base.omegadottable.Var1,1,1)];
states.base.omegaddottable = smoothdata(timetable(time',states.base.omegaddot/parameters.dt));
max_lin_jerk = max(states.base.xddottable.Var1);
max_ang_jerk = max(states.base.omegaddottable.Var1);
max_ang_vel_arm = squeeze(max(max(states.arm.qdottable.Var1)));
states.arm.qddot = [zeros([1,4,3]);diff(states.arm.qdottable.Var1,1,1)];
states.arm.qddottable = smoothdata(timetable(time',(states.arm.qddot/parameters.dt)));
max_ang_accel_arm = squeeze(max(max(states.arm.qddottable.Var1)));

% rmse = sqrt(mean(residuals.^2,"all"));

comp_time
rmse_pos
rmse_rot
max_lin_vel
max_ang_vel
max_ang_vel_arm
max_lin_accel
max_ang_accel
max_ang_accel_arm
% max_lin_jerk
% max_ang_jerk
% max_ang_jerk_arm


% pause(1)

% %% Plot desired vs actual
% figure
% plot(ee_desired(1,:),ee_desired(2,:),end_effector_pose(1,:),end_effector_pose(2,:))
% pause(1)
% 
% %% RMSE
% residuals = ee_desired-end_effector_pose;
% rmse = sqrt(mean(residuals.^2,"all"));
% mean_time = mean(timer);
% 
% %% Remove first point
% x = x(2:end);
% theta1 = theta1(2:end);
% theta2 = theta2(2:end);
% 
% %% Velocity
% time = linspace(0,step_size*dt,step_size);
% 
% dx = diff(x);
% dtheta1 = diff(theta1);
% dtheta2 = diff(theta2);
% 
% dxdt = dx/dt;
% dtheta1dt = dtheta1/dt;
% dtheta2dt = dtheta2/dt;
% 
% max_vel = [max(abs(dxdt)) max([abs(dtheta1dt) abs(dtheta2dt)])];
% 
% %% Acceleration
% dx2 = diff(dxdt);
% dtheta12 = diff(dtheta1dt);
% dtheta22 = diff(dtheta2dt);
% 
% dx2dt2 = dx2/dt;
% dtheta12dt2 = dtheta12/dt;
% dtheta22dt2 = dtheta22/dt;
% 
% max_acc = [max(abs(dx2dt2)) max([abs(dtheta12dt2) abs(dtheta22dt2)])];
% 
% %% Jerk
% dx3 = diff(dx2dt2);
% dtheta13 = diff(dtheta12dt2);
% dtheta23 = diff(dtheta22dt2);
% 
% dx3dt3 = dx3/dt;
% dtheta13dt3 = dtheta13/dt;
% dtheta23dt3 = dtheta23/dt;
% 
% max_jer = [max(abs(dx3dt3)) max([abs(dtheta13dt3) abs(dtheta23dt3)])];
% 
% %% Metrics
% str
% rmse
% mean_time
% max_vel
% max_acc
% max_jer