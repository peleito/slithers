function vector = manifold_to_vector(manifold)
% manifold_to_vector 
% Summary of this function goes here
% Detailed explanation goes here
algebra = manifold_to_algebra(manifold);
vector = algebra_to_vector(algebra);
end