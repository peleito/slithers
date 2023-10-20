figure('units','normalized','outerposition',[0 0 1 1])
subplot(1,3,1)
plotTransforms(pose_goals_helix)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Vertical Helix','FontSize',titlesize/2,'Interpreter','latex')

subplot(1,3,2)
plotTransforms(pose_goals_sine)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Sine Wave','FontSize',titlesize/2,'Interpreter','latex')

subplot(1,3,3)
plotTransforms(pose_goals_horizontal)
set(gca,'fontsize',fontsize/2)
xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
xlim([-3,3])
ylim([-3,3])
zlim([0,1])
axis equal
title('Horizontal Helix','FontSize',titlesize/2,'Interpreter','latex')

% % subplot(2,2,4)
% % plotTransforms(pose_goals_spiral)
% % set(gca,'fontsize',fontsize/2)
% % xlabel('x (m)','FontSize',labelsize/2,'Interpreter','latex')
% % ylabel('y (m)','FontSize',labelsize/2,'Interpreter','latex')
% % zlabel('z (m)','FontSize',labelsize/2,'Interpreter','latex')
% % xlim([-3,3])
% % ylim([-3,3])
% % zlim([0,1])
% % axis equal
% % title('Spiral Wave','FontSize',titlesize/2,'Interpreter','latex')

if save
    pause(5)
    export_fig test_paths.png -transparent -native
    pause(5)
    close
end