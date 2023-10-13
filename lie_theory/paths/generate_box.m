function poses = generate_box(a,dt,time)
%generate_box Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
x = t;
y = t;
z = ones([length(t),1]);
translation = [x',y',z];

rx = zeros([length(t),1]);
ry = zeros([length(t),1]);
rz = ones([length(t),1]);
rotation_vecs = [rx,ry,rz];

for i = 1:1:length(t)
    rotation_mat(i) = so3(rotvec2mat3d(rotation_vecs(i,:)));
end

plotTransforms(translation,rotation_mat)

poses = se3(rotation_mat,translation);

% plotTransforms(poses)
% plot3(x,y,z)

end