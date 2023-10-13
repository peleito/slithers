function tau_joints = get_tau_joints(screws,d_state,dof,dt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
tau_joints = manifold_to_vector(step_forward(screws,d_state,dof,dt));
end