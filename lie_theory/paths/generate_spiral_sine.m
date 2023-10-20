function poses = generate_spiral_sine(a,b,dt,time)
%generate_helix Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
x = t+sin(a*t);
y = cos(a*t);
z = b*sin(t)+3*b;
translation = [x',y',z'];

rx = zeros([length(t),1]);
ry = zeros([length(t),1]);
rz = zeros([length(t),1]);
rz = -pi/4*ones([length(t),1]);
rotation_vecs = [rx,ry,rz];

for i = 1:1:length(t)
    rotation_mat(i) = so3(rotvec2mat3d(rotation_vecs(i,:))*[-1,0,0;0,0,1;0,1,0]);
end


% plotTransforms(translation,rotation_mat)

poses = se3(rotation_mat,translation);
% poses = poses(int32(rand()*length(poses)))

% plotTransforms(poses,'FrameSize',1)
% plot3(x,y,z)

end