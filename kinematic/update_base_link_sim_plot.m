function end_effector_pose = update_base_link_sim_plot(handles_plot, x, theta1, theta2, plot_step, end_effector_pose, ee_d)
    global L1 L2 base_size

    % Base center coordinate
    x0 = x;
    y0 = 0;

    % Link1's end coordinate
    x1 = x0 + L1 * cos(theta1);
    y1 = y0 + L1 * sin(theta1);

    % Link2's end coordinate
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);

    end_effector_pose(1,plot_step) = x2;
    end_effector_pose(2,plot_step) = y2;

    % Rectangle base
    x_base = [x - .5 * base_size, x + .5 * base_size, x + .5 * base_size, x - .5 * base_size, x - .5 * base_size];
    y_base = [-.5 * base_size, -.5 * base_size, .5 * base_size, .5 * base_size, -.5 * base_size];
    set(handles_plot(1), 'XData', x_base);
    set(handles_plot(1), 'YData', y_base);

    % Links
    set(handles_plot(2), 'XData', [x0, x1, x2]);
    set(handles_plot(2), 'YData', [y0, y1, y2]);

    % Link nodes
    set(handles_plot(3), 'XData', [x0, x1, x2]);
    set(handles_plot(3), 'YData', [y0, y1, y2]);

    % Reference line
    set(handles_plot(4), 'XData', [-2.5, 2.5]);
    set(handles_plot(4), 'YData', [0, 0]);

    % End effector trajectory
    set(handles_plot(5), 'XData', end_effector_pose(1,1:plot_step));
    set(handles_plot(5), 'YData', end_effector_pose(2,1:plot_step));

    set(handles_plot(6), 'XData', ee_d(1,:));
    set(handles_plot(6), 'YData', ee_d(2,:));

end