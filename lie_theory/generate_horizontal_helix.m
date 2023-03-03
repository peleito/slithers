function poses = generate_horizontal_helix(a,b,dt,time)
%generate_helix Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
x = b*t+1;
y = a*sin(t)+0.5;
z = 0.5*a*cos(t)+a+0.25;
translation = [x',y',z'];

rx = zeros([length(t),1]);
ry = zeros([length(t),1]);
rz = zeros([length(t),1]);
rotation_vecs = [rx,ry,rz];

for i = 1:1:length(t)
    rotation_mat(i) = so3(rotvec2mat3d(rotation_vecs(i,:))*[0,0,1;1,0,0;0,1,0]);
end

% plotTransforms(translation,rotation_mat)

poses = se3(rotation_mat,translation);
% poses = poses(int32(rand()*length(poses)))

% plotTransforms(poses)
% plot3(x,y,z)

end