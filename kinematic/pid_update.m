function [x, theta1, theta2, error, xdot, theta1dot, theta2dot, u] = pid_update(xk, theta1, theta2, x_des, theta1_des, theta2_des, error, xdot, theta1dot, theta2dot)


    Kc = 100;
    Pc = 4.5;
    
    K_b = 5*[0.95*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for the base
    K_t1 = 5*[0.5*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for theta 1
    K_t2 = 1*[0.5*Kc, 0.2*Pc,Pc/0.08]; %Kp Ki Kd for theta 2
    
    xcur = xk;
    theta1_cur = theta1;
    theta2_cur = theta2;
    pid = PID(xcur, theta1_cur, theta2_cur, x_des, theta1_des, theta2_des, K_b, K_t1, K_t2, error, xdot, theta1dot, theta2dot);
    
    
    pid = pid.compute_output();
    pid = pid.update_x_k();
    
    x = pid.x_k(1,1);
    theta1 = pid.x_k(2,1);
    theta2 = pid.x_k(3,1);
    error = pid.previous_error;
    xdot = pid.x_k(4,1);
    theta1dot = pid.x_k(5,1);
    theta2dot = pid.x_k(6,1);
    u = pid.B*pid.u_k;

    pause(0.01)


end