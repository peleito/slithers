function end_effector_pose = forward_kinematics(state)
    global L1 L2

    x = state(1);
    theta1 = state(2);
    theta2 = state(3);

    % Base center coordinate
    x0 = x;
    y0 = 0;

    % Link1's end coordinate
    x1 = x0 + L1 * cos(theta1); %(1-pow_p(theta1,2)/2+pow_p(theta1,4)/24-pow_p(theta1,6)/720); %cos(theta1);
    y1 = y0 + L1 * sin(theta1); %(theta1-pow_p(theta1,3)/6+pow_p(theta1,5)/120-pow_p(theta1,7)/5040); %sqrt(1-(1-theta1^2/2+theta1^4/24-theta1^6/720)^2); %sin(theta1);

    % Link2's end coordinate
    thetac = theta1+theta2;
    x2 = x1 + L2 * cos(thetac); %(1-thetac^2/factorial(2)+thetac^4/factorial(4)-thetac^6/factorial(6));
    y2 = y1 + L2 * sin(thetac); %sqrt(1-(1-thetac^2/factorial(2)+thetac^4/factorial(4)-thetac^6/factorial(6))^2);

    end_effector_pose = [x2;
                         y2];

end