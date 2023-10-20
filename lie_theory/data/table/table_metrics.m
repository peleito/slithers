comp_time = mean(timer);
max_lin_vel = max(states.base.xtable.Var1);
max_ang_vel = max(states.base.omegatable.Var1);
max_lin_accel = max(states.base.xdottable.Var1);
max_ang_accel = max(states.base.omegadottable.Var1);
states.base.xddot = [zeros([1,num_paths]);diff(states.base.xdottable.Var1,1,1)];
states.base.xddottable = smoothdata(timetable(time',states.base.xddot/parameters.dt));
states.base.omegaddot = [zeros([1,num_paths]);diff(states.base.omegadottable.Var1,1,1)];
states.base.omegaddottable = smoothdata(timetable(time',states.base.omegaddot/parameters.dt));
max_lin_jerk = max(states.base.xddottable.Var1);
max_ang_jerk = max(states.base.omegaddottable.Var1);

max_ang_vel_arm = squeeze(max(max(states.arm.qdottable.Var1)));
states.arm.qddot = [zeros([1,6,num_paths]);diff(states.arm.qdottable.Var1,1,1)];
states.arm.qddottable = smoothdata(timetable(time',(states.arm.qddot/parameters.dt)));
max_ang_accel_arm = squeeze(max(max(states.arm.qddottable.Var1)));
states.arm.qdddot = [zeros([1,6,num_paths]);diff(states.arm.qddottable.Var1,1,1)];
states.arm.qdddottable = smoothdata(timetable(time',(states.arm.qdddot/parameters.dt)));
max_ang_jerk_arm = squeeze(max(max(states.arm.qdddottable.Var1)));

% rmse = sqrt(mean(residuals.^2,"all"));

comp_time
rmse_pos
rmse_rot
max_lin_vel
max_ang_vel
max_ang_vel_arm
max_lin_accel
max_ang_accel
max_ang_accel_arm
max_lin_jerk
max_ang_jerk
max_ang_jerk_arm