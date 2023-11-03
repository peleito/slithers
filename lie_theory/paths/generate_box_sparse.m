function poses = generate_box_sparse(a,dt,time)
%generate_box Summary of this function goes here
%   Detailed explanation goes here
t = 0:dt:time;
divisions = floor(length(t)/8);
% divisions = 10

x_c = [0,a/2,a/2,-a/2,-a/2,a/2,a/2,0,0];
y_c = [a/2,a/2,-a/2,-a/2,-a/2,-a/2,a/2,a/2,a/2];
z_c = [a/4,a/4,a/4,a/4,a/2,a/2,a/2,a/2,a/4];

for step = 1:8
    for substep = 1:divisions
        x((step-1)*divisions+substep) = (x_c(step+1)-x_c(step))/divisions*substep+x_c(step);
        y((step-1)*divisions+substep) = (y_c(step+1)-y_c(step))/divisions*substep+y_c(step);
        z((step-1)*divisions+substep) = (z_c(step+1)-z_c(step))/divisions*substep+z_c(step);
    end
end
x(end+1) = x(1);
y(end+1) = y(1);
z(end+1) = z(1);

translation = [x',y',z'];

rx = zeros([length(x),1]);
ry = zeros([length(x),1]);
rz = ones([length(x),1]);
rotation_vecs = [rx,ry,rz];

for i = 1:1:length(x)
    rotation_mat(i) = so3(rotvec2mat3d(rotation_vecs(i,:)));
end

% plotTransforms(translation,rotation_mat)

poses = se3(rotation_mat,translation);

% plotTransforms(poses)
% plot3(x,y,z)

end