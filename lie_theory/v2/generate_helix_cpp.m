function poses = generate_helix_cpp(a,b,dt,time)
%generate_helix Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
x = a*cos(t);
y = a*sin(t);
z = b*cos(pi/2*t);
translation = [x',y',z'];

rx = zeros([length(t),1]);
ry = zeros([length(t),1]);
rz = t'+pi;
rotation_vecs = [rx,ry,rz];
rotation_mat = zeros([3,3,length(t)]);

for i = 1:1:length(t)
    rotation_mat(:,:,i) = rotvec2mat3d(rotation_vecs(i,:));
end

% plotTransforms(translation,rotation_mat)

poses = zeros([4,4,length(t)]);
poses(1:3,1:3,:) = rotation_mat;
poses(1:3,4,:) = reshape(translation',[3,1,length(t)]);
poses(4,4,:) = ones([length(t),1]);

% plotTransforms(poses)

end