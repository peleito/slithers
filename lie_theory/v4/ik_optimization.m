function [states_d] = ik_optimization(current_state, current_vel, pose_goal, parameters)
%     global L1 L2 stateMin stateMax

    lambda_e = parameters.lambda_e;
    lambda_j = parameters.lambda_j;
    lambda_v = parameters.lambda_v;
    dt = parameters.dt;

%     ee_desired;
%     xk = x(plot_step);
%     if plot_step-1>0
%         xk1 = x(plot_step-1);
% 
%         if plot_step-2>0
%             xk2 = x(plot_step-2);
% 
%             if plot_step-3>0
%                 xk3 = x(plot_step-3);
%             else
%                 xk3 = 0;
%             end
%         else
%             xk2 = 0;
%             xk3 = 0;
%         end
%     else
%         xk1 = 0;
%         xk2 = 0;
%         xk3 = 0;
%     end
% 
%     theta1k = theta1(plot_step);
%     theta2k = theta2(plot_step);

    screws = parameters.screws; % screw joints for mobile manipulator (n rows of [omegax,omegay,omegaz,x,y,z])
    config_state = parameters.config_state; % zero state pose for mobile manipulator (se3 matrix, pose at q = [0])
%     adj = parameters.adjoint; % adjoint matrix for mobile manipulator (se3 matrix, current pose)
    stateMin = parameters.stateMin;
    stateMax = parameters.stateMax;
    base_pose = parameters.base_pose;
    dof.base = parameters.base_dof;
    dof.arm = parameters.arm_dof;
    dof.total = parameters.total_dof;

    tau_poses = manifold_to_vector(inv(base_pose)*pose_goal*inv(config_state));
    tau_joints = @(d_state) manifold_to_vector(step_forward(screws,d_state));

%     fun = @(state_d) lambda_e*norm(ee_desired-forward_kinematics(state_d)) + norm(lambda_v.*(state_d-[xk theta1k theta2k])) + lambda_j*abs((5/2*state_d(1)-9*xk+12*xk1-7*xk2+3/2*xk3)/(dt^3)); %norm(1*state_d(1)-3*xk+3*xk1-1*xk2); % norm(5/2*state_d(1)-9*xk+12*xk1-7*xk2+3/2*xk3);
%     fun = @(d_state) lambda_e*norm(manifold_to_vector(step_forward(screws,d_state))-tau_poses) + norm(lambda_v.*d_state);
%     fun = @(d_state) lambda_e*norm(get_tau_joints(screws,d_state,dof)-tau_poses) + norm(lambda_v.*d_state);
    fun = @(d_state) norm(lambda_e.*(get_tau_joints(screws,d_state,dof)-tau_poses)) + norm(lambda_v.*d_state);
    x0 = current_state;
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = stateMin;
    ub = stateMax;
    nonlcon = [];
    
    opts = optimoptions('fmincon','Algorithm','interior-point');%,'StepTolerance',1e-50);
    states_d = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,opts);

%     x_d = state(1);
%     theta1_d = state(2);
%     theta2_d = state(3);

end