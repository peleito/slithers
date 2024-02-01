%% Test publishing trajectory_msgs/JointTrajectory
clear
close all
% clc

arm_pub = rospublisher("/pos_joint_traj_controller/command","trajectory_msgs/JointTrajectory","DataFormat","struct");

r = rosrate(1);

% pause(5);
%% Main

for plot_step = 1:2
   
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
    positions = [pi/3;-pi/2;0;-pi/9;pi/2;0];
    positions = [0;0;0;0;0;0];
    ur_positions = [positions(3);positions(2);positions(1);positions(4:6)];
    point.Positions = ur_positions; % make sure to switch index 3 and 1
    point.Velocities = zeros([length(ur_positions),1]);
    point.Accelerations = zeros([length(ur_positions),1]);
    point.Effort = zeros([length(ur_positions),1]);
    point.TimeFromStart.Sec = int32(0);
    point.TimeFromStart.Nsec = int32(1e+8);
    arm_msg.Points = point;

    send(arm_pub,arm_msg);

    waitfor(r);
end
