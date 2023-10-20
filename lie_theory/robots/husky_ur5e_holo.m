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

S3_w = [0,0,0]; % base linear
S3_v = [0,1,0];
S3 = [S3_w,S3_v];

S4_w = [0,0,1]; % shoulder pan
S4_v = -cross(S4_w,[0,0,0.163]+husky_base_to_ur_base);
S4 = [S3_w,S4_v];

S5_w = [0,1,0]; % shoulder lift
S5_v = -cross(S5_w,[0,0,0.163]+husky_base_to_ur_base);
S5 = [S5_w,S5_v];

S6_w = [0,1,0]; % elbow
S6_v = -cross(S6_w,[0.425,0,0.162]+husky_base_to_ur_base);
S6 = [S6_w,S6_v];

S7_w = [0,1,0]; % wrist 1
S7_v = -cross(S7_w,[0.817,0.133,0.162]+husky_base_to_ur_base);
S7 = [S7_w,S7_v];

S8_w = [0,0,-1]; % wrist 2
S8_v = -cross(S8_w,[0.817,0.133,0.063]+husky_base_to_ur_base); 
S8 = [S8_w,S8_v];

S9_w = [0,1,0]; % wrist 3
S9_v = -cross(S9_w,[0.817,0.233,0.063]+husky_base_to_ur_base);
S9 = [S9_w,S9_v];

config_state = [-1,0,0,0.817+Lbx;
                           0,0,1,0.233+Lby;
                           0,1,0,0.063+Lbz;
                           0,0,0,1]; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])

stateMin = [-2*pi,-5,-5,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi]';
stateMax = [2*pi,5,5,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi]';

screws = [S1; % base angular velocity
                     S2; % base linear velocity
                     S3; % shoulder pan
                     S4; % shoulder lift
                     S5; % elbow
                     S6; % wrist 1
                     S7; % wrist 2
                     S8;
                     S9]; % wrist 3 screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])

base_dof = 3;
arm_dof = 6;