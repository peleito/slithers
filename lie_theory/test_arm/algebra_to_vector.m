function vector = algebra_to_vector(algebra)
% algebra_to_vector 
% Summary of this function goes here
% Detailed explanation goes here
omega_skew = algebra(1:3,1:3);
rho = algebra(1:3,4);
omega = [omega_skew(3,2);
         omega_skew(1,3);
         omega_skew(2,1)];
vector = [omega;rho];
end