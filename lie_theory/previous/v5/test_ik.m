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
                     S6; % wrist 1
                     S7; % wrist 2
                     S8]; % wrist 3 screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

%% Parameters definition
parameters.config_state = [-1,0,0,0.817+Lbx;
                           0,0,1,0.233+Lby;
                           0,1,0,0.063+Lbz;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
% parameters.adjoint = eye(6); % adjoint matrix for mobile manipulator (se3 matrix, current pose)
parameters.stateMin = [-2*pi,-5,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi]';
parameters.stateMax = [2*pi,5,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]';
parameters.dt = 0.1;
parameters.lambda_e = [1,1,1,100000,100000,100000]';
parameters.lambda_j = 0.000001;
% parameters.lambda_v = [10,10,5,5,5,1,1,1]'/; % length of n
parameters.lambda_v = [0,0,0,0,0,0,0,0]'; % length of n
parameters.time = 10;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = 2;
parameters.arm_dof = 6;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;
dof.base = parameters.base_dof;
dof.arm = parameters.arm_dof;
dof.total = parameters.total_dof;
dof2.base = 2;
dof2.arm = 0;
dof2.total = 2;


%% Initialize
% set array of pose goals (4 by 4 by k)
% pose_goals = generate_helix(0.25,0.125,parameters.dt,parameters.time);
% pose_goals = generate_box(0.5,parameters.dt,parameters.steps);
pose_goals = generate_sine(0.25,0.25,parameters.dt,parameters.time);
% set up communication with ROS
% rosIP = '172.27.185.221';   % IP address of ROS-enabled machine  
% rosinit(rosIP,11311); % Initialize ROS connection
% setenv('ROS_MASTER_URI','http://172.27.185.221:11311')
% setenv('ROS_IP','172.27.185.221')
% setenv('ROS_HOSTNAME','172.27.185.221')
% setenv('ROS_MASTER_URI','http://10.0.0.250:11311')
% setenv('ROS_IP','10.0.0.250')
% setenv('ROS_HOSTNAME','10.0.0.250')
% % rosinit('172.27.185.221',11311)
% rosinit
% get current state from robot (joint states)
% arm_sub = rossubscriber("/joint_states","DataFormat","struct"); % sensor_msgs/jointstate.msg
% base_sub = rossubscriber("/ground_truth/husky_footprint","DataFormat",'struct'); % nav_msgs/Odometry
% tool_sub = rossubscriber("/ground_truth/ur_arm_tool0","DataFormat","struct"); % nav_msgs/Odometry
% husky_pub = rospublisher("/husky_velocity_controller/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
% arm_pub = rospublisher("/pos_joint_traj_controller/command","trajectory_msgs/JointTrajectory","DataFormat","struct");

% pause(5);

% r = rosrate(1);

timer = zeros([1,parameters.steps]);
x = zeros([8,1]);
xdot = zeros([8,1]);
dx = zeros([8,1]);
parameters.base_pose = eye(4);
% arm_received = receive(arm_sub,10);
% a = 1
% base_state = receive(base_sub,10);
% a = 2

% pause(5);
%% Main
pose = eye(4);

for plot_step = 1:parameters.steps
    % get current state from robot
%     arm_received = receive(arm_sub,10);
% %     a = 1
%     base_state = receive(base_sub,10);
% %     a = 2
% 
%     arm_state = arm_received.Position;
%     arm_state = arm_state(5:10);
%     arm_state = [arm_state(3);arm_state(2);arm_state(1);arm_state(4:6)];
% 
%     pos = [base_state.Pose.Pose.Position.X,base_state.Pose.Pose.Position.Y,base_state.Pose.Pose.Position.Z];
%     quaternion_b = [base_state.Pose.Pose.Orientation.W,base_state.Pose.Pose.Orientation.X,base_state.Pose.Pose.Orientation.Y,base_state.Pose.Pose.Orientation.Z];
%     orientation = rotvec(quaternion(quaternion_b));
%     x = [pos(1);orientation(3);arm_state];

    % get next pose goal
    ee_desired = tform(pose_goals(plot_step));
% 
%     % set adjoint matrix from current base pose
% %     parameters.adjoint = zeros([6,6]);
%     rotation = rotvec2mat3d(orientation);
% %     pos_skew = [0,-pos(3),pos(2);
% %                 pos(3),0,-pos(1);
% %                 -pos(2),pos(1),0];
% %     parameters.adjoint(1:3,1:3) = rotation;
% %     parameters.adjoint(1:3,4:6) = cross(pos_skew,rotation);
% %     parameters.adjoint(4:6,4:6) = rotation;
% 
% %     parameters.adjoint = eye(6);
%     
%     parameters.base_pose = eye(4);
%     parameters.base_pose(1:3,1:3) = rotation;
%     parameters.base_pose(1:3,4) = pos;
    parameters.base_pose = parameters.base_pose*step_forward(parameters.screws,dx,dof2);
    parameters.base_pose;

    % perform ik optimization
    tic
    dx = ik_optimization(x, xdot, ee_desired, parameters)
    xdot(:,plot_step) = dx';
%     x = dx;
%     dx = ik_optimization(x, xdot, ee_desired, parameters)
%     x = dx;
%     dx = ik_optimization(x, xdot, ee_desired, parameters)
    timed = toc
    timer(plot_step) = timed;
    
    tool_pose(plot_step) = se3(parameters.base_pose*step_forward(parameters.screws,dx,dof)*parameters.config_state);
%     dx(1:2) = x(1:2)+dx(1:2);
    x(1:2) = [0;0];
    x(3:end) = dx(3:end);
% %     dx(plot_step,:) = dx;
%     xdot = dx;
% %     %% send values to robot
%     husky_msg = rosmessage(husky_pub);
%     husky_msg.Linear.X = dx(1);
%     husky_msg.Linear.Y = 0;
%     husky_msg.Linear.Z = 0;
%     husky_msg.Angular.X = 0;
%     husky_msg.Angular.Y = 0;
%     husky_msg.Angular.Z = dx(2);
%     
%     arm_msg = rosmessage(arm_pub);
% %     arm_msg.Header.Seq = uint32(plot_step);
% %     arm_msg.Header.Stamp = rostime('now',"DataFormat","struct");
% %     arm_msg.Header.FrameId = '';
%     arm_msg.JointNames = {'ur_arm_elbow_joint';
%                            'ur_arm_shoulder_lift_joint';
%                            'ur_arm_shoulder_pan_joint';
%                            'ur_arm_wrist_1_joint';
%                            'ur_arm_wrist_2_joint';
%                            'ur_arm_wrist_3_joint'};
% %     point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
%     positions = dx(3:end);
%     ur_positions = [positions(3);positions(2);positions(1);positions(4:6)];
%     point.Positions = ur_positions; % make sure to switch index 3 and 1
%     point.Velocities = zeros([length(ur_positions),1]);
%     point.Accelerations = zeros([length(ur_positions),1]);
%     point.Effort = zeros([length(ur_positions),1]);
%     point.TimeFromStart.Sec = int32(2);
%     point.TimeFromStart.Nsec = int32(0);
%     arm_msg.Points = point;
% 
% %     waitfor(r);
% 
%     send(arm_pub,arm_msg);
%     send(husky_pub,husky_msg);
% 
%     pause(5)
%     
%     tool_state = receive(tool_sub,10);
%     pos_t = [tool_state.Pose.Pose.Position.X,tool_state.Pose.Pose.Position.Y,tool_state.Pose.Pose.Position.Z];
%     quaternion_t = [tool_state.Pose.Pose.Orientation.W,tool_state.Pose.Pose.Orientation.X,tool_state.Pose.Pose.Orientation.Y,tool_state.Pose.Pose.Orientation.Z];
%     orientation_t = rotvec(quaternion(quaternion_t));
%     rotation_t = rotvec2mat3d(orientation_t);
%     tool_pose = eye(4);
%     tool_pose(1:3,1:3) = rotation_t;
%     tool_pose(1:3,4) = pos_t;

%     goal = manifold_to_vector(ee_desired);
    goal = ee_desired;
%     actual = manifold_to_vector(tform(tool_pose(plot_step)));
    actual = tform(tool_pose(plot_step));
    error(:,:,plot_step) = goal-actual;
    error(:,:,plot_step)

%     end_effector_pose = update_base_link_sim_plot(plot_handles, x(plot_step), theta1(plot_step), theta2(plot_step), plot_step, end_effector_pose, ee_desired);
%     pause(0.1)
%     waitfor(r);
%     pause()
end

%% Plot
% figure
% plot(error(1,:))
% hold on
% plot(error(2,:))
% plot(error(3,:))
% plot(error(4,:))
% plot(error(5,:))
% plot(error(6,:))
% legend('rx','ry','rz','x','y','z')
% grid minor
figure
% plot(error(1,:))
hold on
% plot(error(2,:))
% plot(error(3,:))
% plot(error(4,:))
% plot(error(5,:))
% plot(error(6,:))
% legend('rx','ry','rz','x','y','z')
plot(squeeze(error(1,4,:)))
plot(squeeze(error(2,4,:)))
plot(squeeze(error(3,4,:)))
legend('x','y','z')
grid minor

figure
plotTransforms(tool_pose)

figure
plot(xdot(1,:))
hold on
plot(xdot(2,:))
plot(xdot(3,:))
plot(xdot(4,:))
plot(xdot(5,:))
plot(xdot(6,:))
plot(xdot(7,:))
plot(xdot(8,:))

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