function algebra = vector_to_algebra(vector)
% vector_to_algebra 
% Summary of this function goes here
% Detailed explanation goes here
omega_skew = [0,-vector(3),vector(2);
              vector(3),0,-vector(1);
              -vector(2),vector(1),0];
rho = [vector(4);
       vector(5);
       vector(6)];
algebra = zeros([4,4]);
algebra(1:3,1:3) = omega_skew;
algebra(1:3,4) = rho;
end