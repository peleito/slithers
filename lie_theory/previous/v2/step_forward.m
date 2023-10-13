function manifold = step_forward(screws,adjoint,d_state)
% step_forward 
% Summary of this function goes here
% Detailed explanation goes here
pose_joints = eye(4);
for num = 1:1:size(screws,2)
    pose_joints = pose_joints*vector_to_manifold(screws(num,:)',adjoint,d_state(num)); 
end
manifold = pose_joints;