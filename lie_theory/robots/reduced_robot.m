L1 = 0.1;
L2 = 0.1;
L3 = 0.5;
L4 = 0.5;

Lbx = L1;
% Lbx = 0;
Lby = 0.0;
Lbz = 0.0;
% Lbz = 0;
mobile_base_to_arm_base = [Lbx,Lby,Lbz];
robot = eye(4);
robot(1:3,4) = mobile_base_to_arm_base;
 
S1_w = [0,0,0]; % base angular
S1_v = [1,0,0]; % base linear
S1 = [S1_w,S1_v];

S2_w = [0,0,1]; % shoulder pan
S2_v = -cross(S2_w,[0,0,0]+mobile_base_to_arm_base);
S2 = [S2_w,S2_v];

S3_w = [0,1,0]; % shoulder lift
S3_v = -cross(S3_w,[0,0,L2]+mobile_base_to_arm_base);
S3 = [S3_w,S3_v];

S4_w = [0,1,0]; % elbow
S4_v = -cross(S4_w,[0,0,L2+L3]+mobile_base_to_arm_base);
S4 = [S4_w,S4_v];

config_state = [1,0,0,0;
                           0,1,0,0;
                           0,0,1,L1+L2+L3+L4;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])

stateMin = [-1,-2*pi,-2*pi,-2*pi]';
stateMax = [1,2*pi,2*pi,2*pi]';

screws = [S1; % base linear velocity
                     S2; % shoulder pan
                     S3; % shoulder lift
                     S4]; % elbow screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

base_dof = 1;
arm_dof = 3;