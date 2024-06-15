function [x_d, theta1_d, theta2_d] = inverse_kinematics(x, theta1, theta2, plot_step, ee_desired, dt)
    global L1 L2 stateMin stateMax

    [x_d, theta1_d, theta2_d] = ik_optimization(x, theta1, theta2, plot_step, ee_desired(:,plot_step), dt);

end