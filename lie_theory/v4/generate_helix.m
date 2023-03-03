function poses = generate_helix(a,b,dt,time)
%generate_helix Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
x = a*cos(t);
y = a*sin(t);
z = b*t+0.5;
translation = [x',y',z'];

rx = zeros([length(t),1]);
ry = zeros([length(t),1]);
rz = t';
rotation_vecs = [rx,ry,rz];

for i = 1:1:length(t)
    rotation_mat(i) = so3(rotvec2mat3d(rotation_vecs(i,:))*[0,0,1;1,0,0;0,1,0]);
end

% plotTransforms(translation,rotation_mat)

poses = se3(rotation_mat,translation);
% poses = poses(int32(rand()*length(poses)))

plotTransforms(poses)
% plot3(x,y,z)

end