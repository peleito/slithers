figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,3,1)
plot(s_ee_desired(1,:),s_ee_desired(2,:),s_end_effector_pose(1,:),s_end_effector_pose(2,:),'LineWidth',2)
set(gca,'fontsize',24)
xlabel('x (meters)','FontSize',40,'Interpreter','latex')
ylabel('y (meters)','FontSize',40,'Interpreter','latex')
axis square
title('Sine Wave','FontSize',48,'Interpreter','latex')
legend('$\mathbf{p}$','$\mathbf{p}^*$','FontSize',24,'Interpreter','latex')

subplot(1,3,2)
plot(h_ee_desired(1,:),h_ee_desired(2,:),h_end_effector_pose(1,:),h_end_effector_pose(2,:),'LineWidth',2)
set(gca,'fontsize',24)
xlabel('x (meters)','FontSize',40,'Interpreter','latex')
ylabel('y (meters)','FontSize',40,'Interpreter','latex')
axis square
title('Heart','FontSize',48,'Interpreter','latex')
legend('$\mathbf{p}$','$\mathbf{p}^*$','FontSize',24,'Interpreter','latex')

subplot(1,3,3)
plot(c_ee_desired(1,:),c_ee_desired(2,:),c_end_effector_pose(1,:),c_end_effector_pose(2,:),'LineWidth',2)
set(gca,'fontsize',24)
xlabel('x (meters)','FontSize',40,'Interpreter','latex')
ylabel('y (meters)','FontSize',40,'Interpreter','latex')
axis square
title('Crescent','FontSize',48,'Interpreter','latex')
legend('$\mathbf{p}$','$\mathbf{p}^*$','FontSize',24,'Interpreter','latex')

export_fig proposed_paths.png -transparent -native




%% RMSE

residuals = vecnorm(s_ee_desired-s_end_effector_pose);
rmse = sqrt(mean(residuals.^2))

residuals = vecnorm(h_ee_desired-h_end_effector_pose);
rmse = sqrt(mean(residuals.^2))

residuals = vecnorm(c_ee_desired-c_end_effector_pose);
rmse = sqrt(mean(residuals.^2))