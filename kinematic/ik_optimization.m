function [x_d, theta1_d, theta2_d] = ik_optimization(x, theta1, theta2, plot_step, ee_desired, dt)
    global L1 L2 stateMin stateMax

    lambda_error = 2.5;
    lambda_jerk = 0.000001;
    lambda_change = [0.6 0.2 0.2];

    ee_desired;
    xk = x(plot_step);
    if plot_step-1>0
        xk1 = x(plot_step-1);

        if plot_step-2>0
            xk2 = x(plot_step-2);

            if plot_step-3>0
                xk3 = x(plot_step-3);
            else
                xk3 = 0;
            end
        else
            xk2 = 0;
            xk3 = 0;
        end
    else
        xk1 = 0;
        xk2 = 0;
        xk3 = 0;
    end

    theta1k = theta1(plot_step);
    theta2k = theta2(plot_step);

    fun = @(state_d) lambda_error*norm(ee_desired-forward_kinematics(state_d)) + norm(lambda_change.*(state_d-[xk theta1k theta2k])) + lambda_jerk*abs((5/2*state_d(1)-9*xk+12*xk1-7*xk2+3/2*xk3)/(dt^3)); %norm(1*state_d(1)-3*xk+3*xk1-1*xk2); % norm(5/2*state_d(1)-9*xk+12*xk1-7*xk2+3/2*xk3);
    x0 = [xk theta1k theta2k];
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = stateMin;
    ub = stateMax;

    state = fmincon(fun,x0,A,b,Aeq,beq,lb,ub);

    x_d = state(1);
    theta1_d = state(2);
    theta2_d = state(3);

end