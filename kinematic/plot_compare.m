figure('units','normalized','outerposition',[0 0 1 1])

subplot(3,3,1)
plot(time(1:end/2),x_s,time(1:end/2),x_plot_s,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel({'Sine Wave','$x$ (meters)'},'FontSize',40,'Interpreter','latex')
axis square
title('Base','FontSize',48,'Interpreter','latex')
legend('$x$','$x^*$','FontSize',24,'Interpreter','latex')

subplot(3,3,2)
plot(time(1:end/2),theta1_s,time(1:end/2),theta1_plot_s,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_1$ (rad)','FontSize',40,'Interpreter','latex')
axis square
title('Joint 1','FontSize',48,'Interpreter','latex')
legend('$\theta_1$','$\theta^*_1$','FontSize',24,'Interpreter','latex')

subplot(3,3,3)
plot(time(1:end/2),theta2_s,time(1:end/2),theta2_plot_s,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_2$ (rad)','FontSize',40,'Interpreter','latex')
axis square
title('Joint 2','FontSize',48,'Interpreter','latex')
legend('$\theta_2$','$\theta^*_2$','FontSize',24,'Interpreter','latex')


subplot(3,3,4)
plot(time(1:end/2),x_h,time(1:end/2),x_plot_h,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel({'Heart','$x$ (meters)'},'FontSize',40,'Interpreter','latex')
axis square
% title('Base','FontSize',48,'Interpreter','latex')
legend('$x$','$x^*$','FontSize',24,'Interpreter','latex')

subplot(3,3,5)
plot(time(1:end/2),theta1_h,time(1:end/2),theta1_plot_h,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_1$ (rad)','FontSize',40,'Interpreter','latex')
axis square
% title('Joint 1','FontSize',48,'Interpreter','latex')
legend('$\theta_1$','$\theta^*_1$','FontSize',24,'Interpreter','latex')

subplot(3,3,6)
plot(time(1:end/2),theta2_h,time(1:end/2),theta2_plot_h,'LineWidth',2)
set(gca,'fontsize',24)
% xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_2$ (rad)','FontSize',40,'Interpreter','latex')
axis square
% title('Joint 2','FontSize',48,'Interpreter','latex')
legend('$\theta_2$','$\theta^*_2$','FontSize',24,'Interpreter','latex')


subplot(3,3,7)
plot(time(1:end-1),x_c,time(1:end-1),x_plot_c,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel({'Crescent','$x$ (meters)'},'FontSize',40,'Interpreter','latex')
axis square
% title('Base','FontSize',48,'Interpreter','latex')
legend('$x$','$x^*$','FontSize',24,'Interpreter','latex')

subplot(3,3,8)
plot(time(1:end-1),theta1_c,time(1:end-1),theta1_plot_c,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_1$ (rad)','FontSize',24,'Interpreter','latex')
axis square
% title('Joint 1','FontSize',48,'Interpreter','latex')
legend('$\theta_1$','$\theta^*_1$','FontSize',24,'Interpreter','latex')

subplot(3,3,9)
plot(time(1:end-1),theta2_c,time(1:end-1),theta2_plot_c,'LineWidth',2)
set(gca,'fontsize',24)
xlabel('time (sec)','FontSize',40,'Interpreter','latex')
ylabel('$\theta_2$ (rad)','FontSize',40,'Interpreter','latex')
axis square
% title('Joint 2','FontSize',48,'Interpreter','latex')
legend('$\theta_2$','$\theta^*_2$','FontSize',24,'Interpreter','latex')


export_fig proposed_compare.png -transparent -native