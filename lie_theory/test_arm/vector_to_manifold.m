function manifold = vector_to_manifold(vector,adjoint,d_state)
% manifold_to_vector 
% Summary of this function goes here
% Detailed explanation goes here
algebra = vector_to_algebra(adjoint*vector);
manifold = algebra_to_manifold(algebra*d_state);
end