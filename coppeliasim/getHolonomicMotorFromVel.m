function states = getHolonomicMotorFromVel(x,y,omega)
omega = omega/2;
 for i = 1:length(x)	
    front_left(i) = x(i) + -omega(i) + y(i) ;
    front_right(i) =  -omega(i) - x(i) + y(i) ;
    rear_left(i) = x(i) + -omega(i) - y(i) ;
    rear_right(i) =  -omega(i) - x(i) - y(i) ;
 end
 

states = 10*2.398795.*[rear_left; rear_right; front_right; front_left];
end

%part   x           y           z
% body    -0.22       0.175       0.22799
% pad 0   -0.41806    0.37306     0.05417
% pad 1   -0.41806    -0.02216    0.05417
% pad 2   -0.02293    -0.02216    0.05417
% pad 3   -0.02284    0.37306     0.05417

%       pad3 ----0.39522 m------- pad2
%       |                         |
%       0.39522 m                 0.39522 m
%       |                         |
%       pad0 ----0.39522 m------ pad1


%^ x
%|
% -> -y