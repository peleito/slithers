%% Inverse kinematic solver for a mobile manipulator
clear
close all
% clc

%% Screw joints
Lbx = 0.34;
Lby = 0.0;
Lbz = 0.384;
W1 = 109/1000;
W2 = 82/1000;
L1 = 425/1000;
L2 = 392/1000;
H1 = 89/1000;
H2 = 95/1000;
husky_base_to_ur_base = [Lbx,Lby,Lbz];

S1_w = [0,0,1]; % base angular
S1_v = -cross(S1_w,[0,0,0]);
S1 = [S1_w,S1_v];

S2_w = [0,0,0]; % base linear
S2_v = [1,0,0];
S2 = [S2_w,S2_v];

parameters.screws = [S1; % base angular velocity
                     S2]; % base linear velocity screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

%% Parameters definition
parameters.config_state = [1,0,0,0;
                           0,1,0,0;
                           0,0,1,0;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
parameters.adjoint = eye(6); % adjoint matrix for mobile manipulator (se3 matrix, current pose)
parameters.stateMin = [-100,-1000]';
parameters.stateMax = [100,1000]';
parameters.dt = 1;
parameters.lambda_e = 50;
parameters.lambda_j = 0.000001;
parameters.lambda_v = [1,1]'; % length of n
parameters.time = 10;
parameters.steps = parameters.time/parameters.dt;

%% Initialize
% set array of pose goals (4 by 4 by k)
pose_goals = generate_helix(0.5,0.5,parameters.dt,parameters.steps);
% pose_goals = generate_box(0.5,parameters.dt,parameters.steps);
% set up communication with ROS
% rosIP = '172.27.185.221';   % IP address of ROS-enabled machine  
% rosinit(rosIP,11311); % Initialize ROS connection
% setenv('ROS_MASTER_URI','http://172.27.185.221:11311')
% setenv('ROS_IP','172.27.185.221')
% setenv('ROS_HOSTNAME','172.27.185.221')
setenv('ROS_MASTER_URI','http://10.0.0.250:11311')
setenv('ROS_IP','10.0.0.250')
setenv('ROS_HOSTNAME','10.0.0.250')
% rosinit
% get current state from robot (joint states)
% arm_sub = rossubscriber("/joint_states","DataFormat","struct"); % sensor_msgs/jointstate.msg
base_sub = rossubscriber("/ground_truth/husky_footprint","DataFormat",'struct'); % nav_msgs/Odometry
% tool_sub = rossubscriber("/ground_truth/ur_arm_tool0","DataFormat","struct"); % nav_msgs/Odometry
husky_pub = rospublisher("/husky_velocity_controller/cmd_vel","geometry_msgs/Twist","DataFormat","struct");
% arm_pub = rospublisher("/pos_joint_traj_controller/command","trajectory_msgs/JointTrajectory","DataFormat","struct");

r = rosrate(1);

timer = zeros([1,parameters.steps]);
x = zeros([8,1]);
xdot = zeros([8,1]);

pause(5);
%% Main

for plot_step = 1:parameters.steps
    % get current state from robot
%     arm_received = receive(arm_sub,10);
    base_state = receive(base_sub,10);
    
%     arm_state = arm_received.Position;
%     arm_state = arm_state(5:10);
%     arm_state = [arm_state(3);arm_state(2);arm_state(1);arm_state(4:6)];

    pos = [base_state.Pose.Pose.Position.X,base_state.Pose.Pose.Position.Y,base_state.Pose.Pose.Position.Z];
    quaternion_b = [base_state.Pose.Pose.Orientation.W,base_state.Pose.Pose.Orientation.X,base_state.Pose.Pose.Orientation.Y,base_state.Pose.Pose.Orientation.Z];
    orientation = rotvec(quaternion(quaternion_b));
    x = [pos(1);orientation(3)];

    % get next pose goal
    ee_desired = tform(pose_goals(plot_step));

    % set adjoint matrix from current base pose
    parameters.adjoint = zeros([6,6]);
    rotation = rotvec2mat3d(orientation);
    pos_skew = [0,-pos(3),pos(2);
                pos(3),0,-pos(1);
                -pos(2),pos(1),0];
    parameters.adjoint(1:3,1:3) = rotation;
    parameters.adjoint(1:3,4:6) = cross(pos_skew,rotation);
    parameters.adjoint(4:6,4:6) = rotation;

    parameters.adjoint = eye(6);
    
    parameters.base_pose = eye(4);
    parameters.base_pose(1:3,1:3) = rotation;
    parameters.base_pose(1:3,4) = pos;
    parameters.base_pose

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
%     point.TimeFromStart.Sec = int32(1);
%     point.TimeFromStart.Nsec = int32(0);
%     arm_msg.Points = point;

%     waitfor(r);

%     send(arm_pub,arm_msg);
    send(husky_pub,husky_msg);
    
%     tool_state = receive(tool_sub,10);
%     pos_t = [tool_state.Pose.Pose.Position.X,tool_state.Pose.Pose.Position.Y,tool_state.Pose.Pose.Position.Z];
%     quaternion_t = [tool_state.Pose.Pose.Orientation.W,tool_state.Pose.Pose.Orientation.X,tool_state.Pose.Pose.Orientation.Y,tool_state.Pose.Pose.Orientation.Z];
%     orientation_t = rotvec(quaternion(quaternion_t));
%     rotation_t = rotvec2mat3d(orientation_t);
%     tool_pose = eye(4);
%     tool_pose(1:3,1:3) = rotation_t;
%     tool_pose(1:3,4) = pos_t;

    ee_desired
%     tool_pose

%     end_effector_pose = update_base_link_sim_plot(plot_handles, x(plot_step), theta1(plot_step), theta2(plot_step), plot_step, end_effector_pose, ee_desired);
    pause(5)
%     waitfor(r);
%     pause()
end
