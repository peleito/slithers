function states = getHuskyMotorFromVel(x,omega)

R = 1; %0.33/2;
L = 0.555;

speed_right = x./R + omega.*L./(2.*R);
speed_left = x./R - omega.*L./(2.*R);

states = 5*[speed_left'; speed_right'; speed_right'; speed_left'];
end