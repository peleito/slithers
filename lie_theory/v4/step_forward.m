function manifold = step_forward(screws,d_state,dof)
% step_forward 
% Summary of this function goes here
% Detailed explanation goes here
pose_joints = eye(4);
% for num = 1:1:size(screws,1)
%     pose_joints = pose_joints*vector_to_manifold(screws(num,:)',d_state(num)); 
% end

screw_base = zeros([size(screws,2),1]);
for num = 1:1:dof.base
%     pose_joints = pose_joints*vector_to_manifold(screws(num,:)',d_state(num)); 
    screw_base = screw_base+screws(num,:)'*d_state(num);
end
pose_joints = pose_joints*vector_to_manifold(screw_base,1);

for num = dof.base+1:1:dof.total
    pose_joints = pose_joints*vector_to_manifold(screws(num,:)',d_state(num)); 
end


manifold = pose_joints;