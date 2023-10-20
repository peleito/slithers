%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

addpath(genpath('data'))
addpath(genpath('helper'))
addpath(genpath('paths'))
addpath(genpath('robots'))

%% Load Robot
% Load road and set user preferences and parameters here. No need to edit
% other files or code unless for plotting and visualization.

% Load robot file here with screws and dof fully defined
% husky_ur5e;
husky_ur5e_holo;
dt = 0.1; % Time step for the duration <second, R^1>
time = 20; % Simulated duration of the experiment (does not match realtime) <seconds, R^1>
num_paths = 3; % 1-3 to run up to the first 3 paths <unitless, R^1>
lambda_e = [100,100,100,100,100,100]'; % Weights for error in screw vector [rx,ry,rz,x,y,z]' <unitless, R^6>
lambda_j = 0.000001; % Weight for jerk <unitless, R^1>
lambda_v = 10*[0.05,0.05,0.05,0.025,0.025,0.01,0.01,0.01,0.01]'; % Must match the number of screws and dof (n) and be in the same order <unitless, R^n>



%% Parameters definition
parameters.screws = screws;
parameters.config_state = config_state; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.stateMin = stateMin;
parameters.stateMax = stateMax;
parameters.dt = dt;
parameters.lambda_e = lambda_e;
parameters.lambda_j = lambda_j;
% parameters.lambda_v = [10,10,5,5,5,1,1,1]'/; % length of n
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
pose_goals_helix = generate_helix(2.5,0.0125,parameters.dt,parameters.time); %% working
pose_goals_sine = generate_sine(0.25,0.25,parameters.dt,parameters.time); %% working
pose_goals_horizontal = generate_horizontal_helix(0.5,1,parameters.dt,parameters.time); %% working
pose_goals_spiral = generate_spiral_sine(2.5,0.5,parameters.dt,parameters.time);

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
        pose_goals = pose_goals_helix;
    elseif trial == 2
        pose_goals = pose_goals_sine;
    elseif trial == 3
        pose_goals = pose_goals_horizontal;
    else
        pose_goals = pose_goals_spiral;
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
save = false;
time = seconds(0:parameters.dt:parameters.time-parameters.dt);

%% Plot paths
plot_paths
% % figure('units','normalized','outerposition',[0 0 1 1])
% % subplot(1,3,1)
% % plotTransforms(pose_goals_helix)
% % set(gca,'fontsize',fontsize/2)
% % xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
% % ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
% % zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
% % xlim([-3,3])
% % ylim([-3,3])
% % zlim([0,1])
% % axis equal
% % title('Vertical Helix','FontSize',titlesize/2,'Interpreter','latex')
% % 
% % subplot(1,3,2)
% % plotTransforms(pose_goals_sine)
% % set(gca,'fontsize',fontsize/2)
% % xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
% % ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
% % zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
% % xlim([-3,3])
% % ylim([-3,3])
% % zlim([0,1])
% % axis equal
% % title('Sine Wave','FontSize',titlesize/2,'Interpreter','latex')
% % 
% % subplot(1,3,3)
% % plotTransforms(pose_goals_horizontal)
% % set(gca,'fontsize',fontsize/2)
% % xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
% % ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
% % zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
% % xlim([-3,3])
% % ylim([-3,3])
% % zlim([0,1])
% % axis equal
% % title('Horizontal Helix','FontSize',titlesize/2,'Interpreter','latex')
% % 
% % if save
% %     pause(5)
% %     export_fig test_paths.png -transparent -native
% %     pause(5)
% %     close
% % end

%% Plot error
plot_error

% % figure('units','normalized','outerposition',[0 0 1 1])
% % 
% % % vertical helix
% % pose_goal = tform(pose_goals_helix);
% % error_plot.x = squeeze(tool_pose(1,4,:,1)-pose_goal(1,4,1:end-1));
% % error_plot.y = squeeze(tool_pose(2,4,:,1)-pose_goal(2,4,1:end-1));
% % error_plot.z = squeeze(tool_pose(3,4,:,1)-pose_goal(3,4,1:end-1));
% % for m = 1:1:length(tool_pose)
% %     vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,1));
% %     vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
% %     error_plot.rx(m) = vec1(1)-vec2(1);
% %     error_plot.ry(m) = vec1(2)-vec2(2);
% %     error_plot.rz(m) = vec1(3)-vec2(3);
% % end
% % 
% % error_plot.xtable = smoothdata(timetable(time',error_plot.x));
% % error_plot.ytable = smoothdata(timetable(time',error_plot.y));
% % error_plot.ztable = smoothdata(timetable(time',error_plot.z));
% % error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
% % error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
% % error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
% % 
% % residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
% % rmse_pos(1) = sqrt(mean(residuals.^2,'all'));
% % residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
% % rmse_rot(1) = sqrt(mean(residuals.^2,'all'));
% % 
% % subplot(2,3,1)
% % hold on
% % plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
% % legend('x','y','z')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')
% % 
% % 
% % subplot(2,3,4)
% % hold on
% % plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
% % legend('roll','pitch','yaw')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % 
% % % sine wave
% % pose_goal = tform(pose_goals_sine);
% % error_plot.x = squeeze(tool_pose(1,4,:,2)-pose_goal(1,4,1:end-1));
% % error_plot.y = squeeze(tool_pose(2,4,:,2)-pose_goal(2,4,1:end-1));
% % error_plot.z = squeeze(tool_pose(3,4,:,2)-pose_goal(3,4,1:end-1));
% % for m = 1:1:length(tool_pose)
% %     vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,2));
% %     vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
% %     error_plot.rx(m) = vec1(1)-vec2(1);
% %     error_plot.ry(m) = vec1(2)-vec2(2);
% %     error_plot.rz(m) = vec1(3)-vec2(3);
% % end
% % 
% % error_plot.xtable = smoothdata(timetable(time',error_plot.x));
% % error_plot.ytable = smoothdata(timetable(time',error_plot.y));
% % error_plot.ztable = smoothdata(timetable(time',error_plot.z));
% % error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
% % error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
% % error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
% % 
% % residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
% % rmse_pos(2) = sqrt(mean(residuals.^2,'all'));
% % residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
% % rmse_rot(2) = sqrt(mean(residuals.^2,'all'));
% % 
% % subplot(2,3,2)
% % hold on
% % plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
% % legend('x','y','z')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Sine Wave','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,3,5)
% % hold on
% % plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
% % legend('roll','pitch','yaw')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % % horizontal helix
% % pose_goal = tform(pose_goals_horizontal);
% % error_plot.x = squeeze(tool_pose(1,4,:,3)-pose_goal(1,4,1:end-1));
% % error_plot.y = squeeze(tool_pose(2,4,:,3)-pose_goal(2,4,1:end-1));
% % error_plot.z = squeeze(tool_pose(3,4,:,3)-pose_goal(3,4,1:end-1));
% % for m = 1:1:length(tool_pose)
% %     vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,3));
% %     vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
% %     error_plot.rx(m) = vec1(1)-vec2(1);
% %     error_plot.ry(m) = vec1(2)-vec2(2);
% %     error_plot.rz(m) = vec1(3)-vec2(3);
% % end
% % 
% % error_plot.xtable = smoothdata(timetable(time',error_plot.x));
% % error_plot.ytable = smoothdata(timetable(time',error_plot.y));
% % error_plot.ztable = smoothdata(timetable(time',error_plot.z));
% % error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
% % error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
% % error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
% % 
% % residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
% % rmse_pos(3) = sqrt(mean(residuals.^2,'all'));
% % residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
% % rmse_rot(3) = sqrt(mean(residuals.^2,'all'));
% % 
% % subplot(2,3,3)
% % hold on
% % plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
% % legend('x','y','z')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,3,6)
% % hold on
% % plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
% % plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
% % legend('roll','pitch','yaw')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % if save
% %     pause(5)
% %     export_fig error.png -transparent -native
% %     pause(5)
% %     close
% % end

%% Plot base linear states

plot_base_linear

% % figure('units','normalized','outerposition',[0 0 1 1])
% % 
% % states.base.x = squeeze(xdot(2,:,:));
% % states.base.xtable = smoothdata(timetable(time',states.base.x));
% % states.base.xdot = [zeros([1,3]);diff(states.base.xtable.Var1,1,1)];
% % 
% % states.base.xdottable = smoothdata(timetable(time',states.base.xdot/parameters.dt));
% % 
% % % linear x dot
% % subplot(1,2,1)
% % hold on
% % plot(states.base.xtable.Time,states.base.xtable.Var1(:,1),'LineWidth',linewidth)
% % plot(states.base.xtable.Time,states.base.xtable.Var1(:,2),'LineWidth',linewidth)
% % plot(states.base.xtable.Time,states.base.xtable.Var1(:,3),'LineWidth',linewidth)
% % legend('Vertical Helix','Sine Wave','Horizontal Helix')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{x}}$ (m/s)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Base Linear Velocity','FontSize',titlesize,'Interpreter','latex')
% % 
% % % linear x double dot
% % subplot(1,2,2)
% % hold on
% % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,1),'LineWidth',linewidth)
% % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,2),'LineWidth',linewidth)
% % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,3),'LineWidth',linewidth)
% % legend('Vertical Helix','Sine Wave','Horizontal Helix')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\ddot{x}}$ (m/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Base Linear Acceleration','FontSize',titlesize,'Interpreter','latex')
% % 
% % if save
% %     pause(5)
% %     export_fig base_linear.png -transparent -native
% %     pause(5)
% %     close
% % end

%% Plot base angular states

plot_base_angular

% % figure('units','normalized','outerposition',[0 0 1 1])
% % 
% % states.base.omega = squeeze(xdot(1,:,:));
% % states.base.omegatable = smoothdata(timetable(time',states.base.omega));
% % states.base.omegadot = [zeros([1,3]);diff(states.base.omegatable.Var1,1,1)];
% % states.base.omegadottable = smoothdata(timetable(time',states.base.omegadot/parameters.dt));
% % 
% % % angular omega dot
% % subplot(1,2,1)
% % hold on
% % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,1),'LineWidth',linewidth)
% % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,2),'LineWidth',linewidth)
% % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,3),'LineWidth',linewidth)
% % legend('Vertical Helix','Sine Wave','Horizontal Helix')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\omega}$ (rad/s)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Base Angular Velocity','FontSize',titlesize,'Interpreter','latex')
% % 
% % % angular omega double dot
% % subplot(1,2,2)
% % hold on
% % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,1),'LineWidth',linewidth)
% % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,2),'LineWidth',linewidth)
% % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,3),'LineWidth',linewidth)
% % legend('Vertical Helix','Sine Wave','Horizontal Helix')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{\omega}}$ (rad/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Base Angular Acceleration','FontSize',titlesize,'Interpreter','latex')
% % 
% % if save
% %     pause(5)
% %     export_fig base_angular.png -transparent -native
% %     pause(5)
% %     close
% % end

%% Plot joint states

plot_joints
plot_joints_sep

% % figure('units','normalized','outerposition',[0 0 1 1])
% % 
% % states.arm.q = xdot(3:end,:,:);
% % states.arm.qtable = smoothdata(timetable(time',pagetranspose(states.arm.q)));
% % states.arm.qdot = [zeros([1,4,3]);diff(states.arm.qtable.Var1,1,1)];
% % states.arm.qdottable = smoothdata(timetable(time',(states.arm.qdot/parameters.dt)));
% % 
% % % vertical helix
% % subplot(2,3,1)
% % hold on
% % plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,3,4)
% % hold on
% % plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % % sine wave
% % subplot(2,3,2)
% % hold on
% % plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,2)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Sine Wave','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,3,5)
% % hold on
% % plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,2)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % % horizontal helix
% % subplot(2,3,3)
% % hold on
% % plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,3)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,3,6)
% % hold on
% % plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,3)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % 
% % if save
% %     pause(5)
% %     export_fig arm_states.png -transparent -native
% %     pause(5)
% %     close
% % end

%% Table parameters
table_parameters
% % parameters.dt
% % parameters.lambda_e
% % parameters.lambda_v
% % parameters.lambda_j

%% Table metrics
table_metrics
% % comp_time = mean(timer);
% % max_lin_vel = max(states.base.xtable.Var1);
% % max_ang_vel = max(states.base.omegatable.Var1);
% % max_lin_accel = max(states.base.xdottable.Var1);
% % max_ang_accel = max(states.base.omegadottable.Var1);
% % states.base.xddot = [zeros([1,3]);diff(states.base.xdottable.Var1,1,1)];
% % states.base.xddottable = smoothdata(timetable(time',states.base.xddot/parameters.dt));
% % states.base.omegaddot = [zeros([1,3]);diff(states.base.omegadottable.Var1,1,1)];
% % states.base.omegaddottable = smoothdata(timetable(time',states.base.omegaddot/parameters.dt));
% % max_lin_jerk = max(states.base.xddottable.Var1);
% % max_ang_jerk = max(states.base.omegaddottable.Var1);
% % 
% % max_ang_vel_arm = squeeze(max(max(states.arm.qdottable.Var1)));
% % states.arm.qddot = [zeros([1,4,3]);diff(states.arm.qdottable.Var1,1,1)];
% % states.arm.qddottable = smoothdata(timetable(time',(states.arm.qddot/parameters.dt)));
% % max_ang_accel_arm = squeeze(max(max(states.arm.qddottable.Var1)));
% % states.arm.qdddot = [zeros([1,4,3]);diff(states.arm.qddottable.Var1,1,1)];
% % states.arm.qdddottable = smoothdata(timetable(time',(states.arm.qdddot/parameters.dt)));
% % max_ang_jerk_arm = squeeze(max(max(states.arm.qdddottable.Var1)));
% % 
% % % rmse = sqrt(mean(residuals.^2,"all"));
% % 
% % comp_time
% % rmse_pos
% % rmse_rot
% % max_lin_vel
% % max_ang_vel
% % max_ang_vel_arm
% % max_lin_accel
% % max_ang_accel
% % max_ang_accel_arm
% % max_lin_jerk
% % max_ang_jerk
% % max_ang_jerk_arm
