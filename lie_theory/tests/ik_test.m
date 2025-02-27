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
parameters.stateMin = [-100,-100,-pi,-pi,-pi,-pi,-pi,-pi]';
parameters.stateMax = [100,100,pi,pi,pi,pi,pi,pi]';
parameters.dt = 0.1;
parameters.lambda_e = [10,10,10,50,50,50]';
parameters.lambda_j = 0.000001;
parameters.lambda_v = [10,10,5,5,5,1,1,1]'; % length of n
% parameters.lambda_v = [0,0,100,100,100,100,100,100]'; % length of n
parameters.time = 10;
parameters.steps = parameters.time/parameters.dt;
parameters.base_dof = 2;
parameters.arm_dof = 6;
parameters.total_dof = parameters.base_dof+parameters.arm_dof;


%% Initialize
% set array of pose goals (4 by 4 by k)
pose_goals = generate_helix(0.5,0.25,parameters.dt,parameters.steps);
% pose_goals = generate_box(0.5,parameters.dt,parameters.steps);
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
arm_sub = rossubscriber("/joint_states","DataFormat","struct"); % sensor_msgs/jointstate.msg
base_sub = rossubscriber("/ground_truth/husky_footprint","DataFormat",'struct'); % nav_msgs/Odometry
tool_sub = rossubscriber("/ground_truth/ur_arm_tool0","DataFormat","struct"); % nav_msgs/Odometry
husky_pub = rospublisher("/husky_velocity_controller/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
arm_pub = rospublisher("/pos_joint_traj_controller/command","trajectory_msgs/JointTrajectory","DataFormat","struct");



r = rosrate(1);

timer = zeros([1,parameters.steps]);
x = zeros([8,1]);
xdot = zeros([8,1]);

arm_received = receive(arm_sub,10);
base_state = receive(base_sub,10);

%% Main

for plot_step = 1:parameters.steps
    % get current state from robot
    arm_received = receive(arm_sub,10);
%     a = 1
    base_state = receive(base_sub,10);
%     a = 2

    arm_state = arm_received.Position;
    arm_state = arm_state(5:10);
    arm_state = [arm_state(3);arm_state(2);arm_state(1);arm_state(4:6)];

    pos = [base_state.Pose.Pose.Position.X,base_state.Pose.Pose.Position.Y,base_state.Pose.Pose.Position.Z];
    quaternion_b = [base_state.Pose.Pose.Orientation.W,base_state.Pose.Pose.Orientation.X,base_state.Pose.Pose.Orientation.Y,base_state.Pose.Pose.Orientation.Z];
    orientation = rotvec(quaternion(quaternion_b));
    x = [0;0;arm_state];

    % get next pose goal
    ee_desired = tform(pose_goals(plot_step));

    rotation = rotvec2mat3d(orientation);
    parameters.base_pose = eye(4);
    parameters.base_pose(1:3,1:3) = rotation;
    parameters.base_pose(1:3,4) = pos;

    % perform ik optimization
    tic
    dx = ik_optimization(x, xdot, ee_desired, parameters)
    timed = toc
    timer(plot_step) = timed;
%     dx(plot_step,:) = dx;
    xdot = dx;
%     %% send values to robot
    husky_msg = rosmessage(husky_pub);
    husky_msg.Linear.X = dx(1);
    husky_msg.Linear.Y = 0;
    husky_msg.Linear.Z = 0;
    husky_msg.Angular.X = 0;
    husky_msg.Angular.Y = 0;
    husky_msg.Angular.Z = dx(2);
    
    arm_msg = rosmessage(arm_pub);
%     arm_msg.Header.Seq = uint32(plot_step);
%     arm_msg.Header.Stamp = rostime('now',"DataFormat","struct");
%     arm_msg.Header.FrameId = '';
    arm_msg.JointNames = {'ur_arm_elbow_joint';
                           'ur_arm_shoulder_lift_joint';
                           'ur_arm_shoulder_pan_joint';
                           'ur_arm_wrist_1_joint';
                           'ur_arm_wrist_2_joint';
                           'ur_arm_wrist_3_joint'};
%     point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    positions = dx(3:end);
    ur_positions = [positions(3);positions(2);positions(1);positions(4:6)];
    point.Positions = ur_positions; % make sure to switch index 3 and 1
    point.Velocities = zeros([length(ur_positions),1]);
    point.Accelerations = zeros([length(ur_positions),1]);
    point.Effort = zeros([length(ur_positions),1]);
    point.TimeFromStart.Sec = int32(2);
    point.TimeFromStart.Nsec = int32(0);
    arm_msg.Points = point;

%     waitfor(r);

    send(arm_pub,arm_msg);
    send(husky_pub,husky_msg);

    pause(5)
    
    tool_state = receive(tool_sub,10);
    pos_t = [tool_state.Pose.Pose.Position.X,tool_state.Pose.Pose.Position.Y,tool_state.Pose.Pose.Position.Z];
    quaternion_t = [tool_state.Pose.Pose.Orientation.W,tool_state.Pose.Pose.Orientation.X,tool_state.Pose.Pose.Orientation.Y,tool_state.Pose.Pose.Orientation.Z];
    orientation_t = rotvec(quaternion(quaternion_t));
    rotation_t = rotvec2mat3d(orientation_t);
    tool_pose = eye(4);
    tool_pose(1:3,1:3) = rotation_t;
    tool_pose(1:3,4) = pos_t;

    goal = manifold_to_vector(ee_desired)
    actual = manifold_to_vector(tool_pose)
    error = goal-actual

%     end_effector_pose = update_base_link_sim_plot(plot_handles, x(plot_step), theta1(plot_step), theta2(plot_step), plot_step, end_effector_pose, ee_desired);
    pause(5)
    waitfor(r);
%     pause()
end