classdef PID
   properties
       dt = 0.01;
       m1 = 10;
       m2 = 10;
       mb  = 30;
       ml; 
       L1 = 1.2;
       L2 = 1.2;
       I_l1;
       I_l2;
      A;  
      B;
      u_k = [0;
          0;
          0];
      F = 0;
      Tau_1 = 1;
      Tau_2 = 0;
      init =0;
      xk_b = 0;
      theta1 = 0;
      theta2 = 0;
      x_dot = 0;
      theta1_dot = 0;
      theta2_dot = 0;
      intgrl = 0;
      Kp = [0;
          0;
          0];
      Ki = [0;
          0;
          0];
      Kd;
      previous_error = 0;
      outpt = 0;
      desired;
      x_k;
   end
   methods
       function obj = PID(xk_b, theta1, theta2, x_des, theta1_des, theta2_des, K_b, K_t1, K_t2, error, xdot, theta1dot, theta2dot )
          obj.Kp = [K_b(1);
              K_t1(1);
              K_t1(1)];
          obj.Ki = [K_b(2);
              K_t1(2);
              K_t2(2)];
          obj.Kd = [K_b(3);
              K_t1(3);
              K_t2(3)];
          obj.I_l1 = (1/3)*obj.m1*obj.L1^2 + obj.m2*obj.L1^2;
          obj.I_l2 = (1/3)*obj.m2*obj.L2^2;
          obj.ml = obj.m1+ obj.m2 + obj.mb;

          obj.A = [1 0 0 obj.dt 0 0;
                   0 1 0 0 obj.dt 0;
                   0 0 1 0 0  obj.dt;
                   0 0 0 1 0  0;
                   0 0 0 0 1  0;
                   0 0 0 0 0  1
                   ];
          obj.B = [0     0       0;
                   0     0       0;
                   0     0       0;
                   obj.dt/obj.ml 0       0;
                   0     obj.dt/obj.I_l1 0;
                   0     0       obj.dt/obj.I_l2
                   ];
          obj.u_k = [obj.F;
                   obj.Tau_1;
                   obj.Tau_2
                  ];
            obj.xk_b = xk_b;
            obj.theta1 = theta1;
            obj.theta2 = theta2;
            obj.x_dot = xdot;
            obj.theta1_dot = theta1dot;
            obj.theta2_dot = theta2dot;
            obj.x_k = [obj.xk_b;
                       obj.theta1;
                       obj.theta2;
                       obj.x_dot;
                       obj.theta1_dot;
                       obj.theta2_dot
                      ];
            obj.theta1 = theta1;
            obj.theta2 = theta2;
            obj.desired = [x_des;
                           theta1_des;
                           theta2_des;
                            ];
            obj.previous_error = error;

       end
       function obj = compute_output(obj)
            error = obj.desired  - obj.x_k(1:3,:);
            obj.intgrl = obj.intgrl + error*obj.dt;
            derivative = (error - obj.previous_error)/obj.dt;
            obj.u_k = obj.Kp.*error+ obj.Ki.*obj.intgrl + obj.Kd.*derivative;
            obj.previous_error = error;
       end 

       function obj = update_x_k(obj)
           obj.x_k = obj.A*obj.x_k + obj.B*obj.u_k;
           if obj.x_k(4,:) > 8
                obj.x_k(4,:) = 8;
           end
           if obj.x_k(4,:) < -8
                obj.x_k(4,:) = -8;
           end
           if obj.x_k(5,:) > 3*pi
                obj.x_k(5,:) = 3*pi;
           end
           if obj.x_k(5,:) < -3*pi
                obj.x_k(5,:) = -3*pi;
           end
            if obj.x_k(6,:) > 3*pi
                obj.x_k(6,:) = 3*pi;
           end
           if obj.x_k(6,:) < -3*pi
                obj.x_k(6,:) = -3*pi;
           end
       end 
 
   end
end
