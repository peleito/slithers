function handles = create_base_link_sim_plot(l1, l2, base_size, init_x, init_theta1, init_theta2, end_effector_pose, ee_d)

    % Base center coordinate
    x0 = init_x;
    y0 = 0;

    % Link1's end coordinate
    x1 = x0 + l1 * cos(init_theta1);
    y1 = y0 + l1 * sin(init_theta1);

    % Link2's end coordinate
    x2 = x1 + l2 * cos(init_theta1 + init_theta2);
    y2 = y1 + l2 * sin(init_theta1 + init_theta2);
    
    % End effector coordiante
    end_effector_pose(1,:) = x2;
    end_effector_pose(2,:) = y2;

    handles(1) = plot([x0 - .5 * base_size, x0 + .5 * base_size, x0 + .5 * base_size, x0 - .5 * base_size, x0 - .5 * base_size],...
                      [y0 - .5 * base_size, y0 - .5 * base_size, y0 + .5 * base_size, y0 + .5 * base_size, y0 - .5 * base_size], 'LineWidth',1.5);
    axis square
    axis([-3 3 -3 3])
    hold on

    handles(2) = plot([x0, x1, x2], [y0, y1, y2], '-b', 'LineWidth', 2.0);
    handles(3) = plot([x0, x1, x2], [y0, y1, y2], 'o', 'MarkerSize', 5, 'MarkerFaceColor','r');
    handles(4) = plot([-2.5, 2.5], [0, 0], '--g|', 'LineWidth', 1.5, 'MarkerSize', 15);
    handles(5) = plot(end_effector_pose(1,:), end_effector_pose(2,:), '--r', 'LineWidth', 1.0);
    handles(6) = plot(ee_d(1,:),ee_d(2,:), '--r', 'LineWidth', 1.0);

end 