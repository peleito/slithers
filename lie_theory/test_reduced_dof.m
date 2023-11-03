%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

addpath(genpath('data'))
addpath(genpath('helper'))
addpath(genpath('paths'))
addpath(genpath('robots'))
addpath(genpath(fileparts('../github_repo/')))

%% Load Robot
% Load road and set user preferences and parameters here. No need to edit
% other files or code unless for plotting and visualization.

% Load robot file here with screws and dof fully defined
reduced_robot;
dt = 0.1; % Time step for the duration <second, R^1>
time = 20; % Simulated duration of the experiment (does not match realtime) <seconds, R^1>
num_paths = 1; % 1-3 to run up to the first 3 paths <unitless, R^1>
lambda_e = 25*[1,1,1,1,1,1]'; % Weights for error in screw vector [rx,ry,rz,x,y,z]' <unitless, R^6>
lambda_j = 0.001; % Weight for jerk <unitless, R^1>
lambda_v = 10*[0.1,0.1,0.1,0.025]'; % Must match the number of screws and dof (n) and be in the same order <unitless, R^n>

%% Parameters definition
parameters.screws = screws;
parameters.config_state = config_state; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.stateMin = stateMin;
parameters.stateMax = stateMax;
parameters.dt = dt;
parameters.lambda_e = lambda_e;
parameters.lambda_j = lambda_j;
parameters.lambda_v = lambda_v; 
parameters.time = time;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = base_dof; 
parameters.arm_dof = arm_dof;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;

dof.base = parameters.base_dof;
dof.arm = parameters.arm_dof;
dof.total = parameters.total_dof;
dof2.base = parameters.base_dof;
dof2.arm = 0;
dof2.total = parameters.base_dof;

%% Initialize
% set array of pose goals (4 by 4 by k)
pose_goals_box = generate_box_sparse(1,parameters.dt,parameters.time); %% working

timer = zeros([parameters.steps,num_paths]);
x = zeros([dof.total,1]);
xdot = zeros([dof.total,1,num_paths]);
dx = zeros([dof.total,1]);
parameters.base_pose = eye(4);
tool_pose = zeros([4,4,parameters.steps,num_paths]);
error = zeros([4,4,parameters.steps,num_paths]);

%% Main
pose = eye(4);

for trial = 1:1:num_paths
    if trial == 1
        pose_goals = pose_goals_box;
    else
        pose_goals = pose_goals_box;
    end

%     timer = zeros([1,parameters.steps]);
    x = zeros([dof.total,1]);
%     xdot = zeros([6,1,3]);
    dx = zeros([dof.total,1]);
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
        % tool_pose(:,:,plot_step,trial);
        x(1:dof.base) = zeros([dof.base,1]);
        x(dof.base+1:end) = dx(dof.base+1:end);
    
    %     goal = manifold_to_vector(ee_desired);
        goal = ee_desired;
    %     actual = manifold_to_vector(tform(tool_pose(plot_step)));
        actual = tool_pose(:,:,plot_step,trial);
        error(:,:,plot_step,trial) = goal-actual;
        % error(:,:,plot_step,trial);
    
    end

end
%% Plot parameters
linewidth = 2;
fontsize = 20;
labelsize = 28;
titlesize = 40;
save = true;
time = seconds(0:parameters.dt:parameters.time-parameters.dt);

%% Plot paths

figure('units','normalized','outerposition',[0 0 1 1])

plotTransforms(pose_goals_box)
set(gca,'fontsize',fontsize/1)
xlabel('x (m)','FontSize',labelsize/1,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/1,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/1,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Box Path','FontSize',titlesize/1,'Interpreter','latex')

if save
    pause(5)
    export_fig test_paths.png -transparent -native
    pause(5)
    close
end

%% Plot error

figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(2,3);

% proposed method
pose_goal = tform(pose_goals_box);
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

% subplot(2,num_paths,1)
nexttile(tcl,1)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
% legend('x','y','z')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Proposed Method','FontSize',titlesize,'Interpreter','latex')
% ylim([-0.01,0.01])


% subplot(2,num_paths,num_paths+1)
nexttile(tcl,4)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
% legend('roll','pitch','yaw')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
% ylim([-0.01,0.01])


% method 1
pose_goal = tform(pose_goals_box);
error_plot.x = squeeze(tool_pose(1,4,:,1)-pose_goal(1,4,1:end-1)); % change position 4 of tool_pose to other method
error_plot.y = squeeze(tool_pose(2,4,:,1)-pose_goal(2,4,1:end-1)); % change position 4 of tool_pose to other method
error_plot.z = squeeze(tool_pose(3,4,:,1)-pose_goal(3,4,1:end-1)); % change position 4 of tool_pose to other method
for m = 1:1:length(tool_pose)
    vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,1)); % change position 4 of tool_pose to other method
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

% subplot(2,num_paths,2)
nexttile(tcl,2)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
% legend('x','y','z')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Method 1','FontSize',titlesize,'Interpreter','latex')
% ylim([-0.01,0.01])

% subplot(2,num_paths,num_paths+2)
nexttile(tcl,5)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
% legend('roll','pitch','yaw')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
% ylim([-0.01,0.01])

% method 2
pose_goal = tform(pose_goals_box);
error_plot.x = squeeze(tool_pose(1,4,:,1)-pose_goal(1,4,1:end-1)); % change position 4 of tool_pose to other method
error_plot.y = squeeze(tool_pose(2,4,:,1)-pose_goal(2,4,1:end-1)); % change position 4 of tool_pose to other method
error_plot.z = squeeze(tool_pose(3,4,:,1)-pose_goal(3,4,1:end-1)); % change position 4 of tool_pose to other method
for m = 1:1:length(tool_pose)
    vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,1)); % change position 4 of tool_pose to other method
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

% subplot(2,num_paths,3)
nexttile(tcl,3)
hold on
plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
legend('x','y','z','Location','eastoutside')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Method 2','FontSize',titlesize,'Interpreter','latex')
% ylim([-0.01,0.01])

% subplot(2,num_paths,num_paths+3)
nexttile(tcl,6)
hold on
plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
legend('roll','pitch','yaw','Location','eastoutside')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
% ylim([-0.01,0.01])

if save
    pause(5)
    export_fig error.png -transparent -native
    pause(5)
    close
end

%% Plot base linear states

figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(1,2);

states.base.x = squeeze(xdot(1,:,:)); % setup results for other methods
states.base.xtable = smoothdata(timetable(time',states.base.x'));
states.base.xdot = [zeros([1,num_paths]);diff(states.base.xtable.Var1,1,1)]; 

states.base.xdottable = smoothdata(timetable(time',states.base.xdot/parameters.dt));

% linear x dot
% subplot(1,2,1)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.xtable.Time,states.base.xtable.Var1(:,i),'LineWidth',linewidth) % plot results for other method
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{x}}$ (m/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Linear Velocity','FontSize',titlesize,'Interpreter','latex')

% linear x double dot
% subplot(1,2,2)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\ddot{x}}$ (m/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Linear Acceleration','FontSize',titlesize,'Interpreter','latex')

hL = legend('Proposed Method','Method 1','Method 2');
hL.Layout.Tile = 'East';

if save
    pause(5)
    export_fig base_linear.png -transparent -native
    pause(5)
    close
end

%% Plot joint states

% plot_joints
% plot_joints_sep

figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(2,3);

states.arm.q = xdot(dof.base+1:end,:,:);
states.arm.qtable = smoothdata(timetable(time',pagetranspose(states.arm.q)));
states.arm.qdot = [zeros([1,3,num_paths]);diff(states.arm.qtable.Var1,1,1)];
states.arm.qdottable = smoothdata(timetable(time',(states.arm.qdot/parameters.dt)));

% proposed method
% subplot(2,num_paths,1)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Proposed Method','FontSize',titlesize,'Interpreter','latex')

% method 1
% subplot(2,num_paths,2)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth) % update to results from method 1
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Method 1','FontSize',titlesize,'Interpreter','latex')

% method 2
% subplot(2,num_paths,3)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth) % update to results from method 2
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Method 2','FontSize',titlesize,'Interpreter','latex')

% proposed method
% subplot(2,num_paths,num_paths+1)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% method 1
% subplot(2,num_paths,num_paths+2)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth) % update to results from method 1
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% method 2
% subplot(2,num_paths,num_paths+3)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth) % update to results from method 2
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

hL = legend('Shoulder Pan','Shoulder Lift','Elbow');
hL.Layout.Tile = 'East';

if save
    pause(5)
    export_fig arm_states.png -transparent -native
    pause(5)
    close
end

%% Table parameters
table_parameters

%% Table metrics
% table_metrics_nh
% table_metrics_h

comp_time = mean(timer);
max_lin_vel = max(abs(states.base.xtable.Var1));
% max_ang_vel = max(abs(states.base.omegatable.Var1));
max_lin_accel = max(abs(states.base.xdottable.Var1));
% max_ang_accel = max(abs(states.base.omegadottable.Var1));
states.base.xddot = [zeros([1,num_paths]);diff(states.base.xdottable.Var1,1,1)];
states.base.xddottable = smoothdata(timetable(time',states.base.xddot/parameters.dt));
% states.base.omegaddot = [zeros([1,num_paths]);diff(states.base.omegadottable.Var1,1,1)];
% states.base.omegaddottable = smoothdata(timetable(time',states.base.omegaddot/parameters.dt));
max_lin_jerk = max(abs(states.base.xddottable.Var1));
% max_ang_jerk = max(abs(states.base.omegaddottable.Var1));

max_ang_vel_arm = squeeze(max(max(abs(states.arm.qdottable.Var1))));
states.arm.qddot = [zeros([1,3,num_paths]);diff(states.arm.qdottable.Var1,1,1)];
states.arm.qddottable = smoothdata(timetable(time',(states.arm.qddot/parameters.dt)));
max_ang_accel_arm = squeeze(max(max(abs(states.arm.qddottable.Var1))));
states.arm.qdddot = [zeros([1,3,num_paths]);diff(states.arm.qddottable.Var1,1,1)];
states.arm.qdddottable = smoothdata(timetable(time',(states.arm.qdddot/parameters.dt)));
max_ang_jerk_arm = squeeze(max(max(abs(states.arm.qdddottable.Var1))));

% rmse = sqrt(mean(residuals.^2,"all"));

comp_time
rmse_pos
rmse_rot
max_lin_vel
% max_ang_vel
max_ang_vel_arm
max_lin_accel
% max_ang_accel
max_ang_accel_arm
max_lin_jerk
% max_ang_jerk
max_ang_jerk_arm