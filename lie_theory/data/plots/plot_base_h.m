figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(2,3);

states.base.omega = squeeze(xdot(1,:,:));
states.base.omegatable = smoothdata(timetable(time',states.base.omega));
states.base.omegadot = [zeros([1,num_paths]);diff(states.base.omegatable.Var1,1,1)];
states.base.omegadottable = smoothdata(timetable(time',states.base.omegadot/parameters.dt));

% angular omega dot
% subplot(2,2,1)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\omega}$ (rad/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Angular Velocity','FontSize',titlesize,'Interpreter','latex')

states.base.x = squeeze(xdot(2,:,:));
states.base.xtable = smoothdata(timetable(time',states.base.x));
states.base.xdot = [zeros([1,num_paths]);diff(states.base.xtable.Var1,1,1)];

states.base.xdottable = smoothdata(timetable(time',states.base.xdot/parameters.dt));

% linear x dot
% subplot(2,2,3)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.xtable.Time,states.base.xtable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{x}}$ (m/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Forward Velocity','FontSize',titlesize,'Interpreter','latex')

states.base.y = squeeze(xdot(3,:,:));
states.base.ytable = smoothdata(timetable(time',states.base.y));
states.base.ydot = [zeros([1,num_paths]);diff(states.base.ytable.Var1,1,1)];

states.base.ydottable = smoothdata(timetable(time',states.base.ydot/parameters.dt));

% linear y dot
% subplot(2,2,3)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xtable.Time,states.base.xtable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.ytable.Time,states.base.ytable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{y}}$ (m/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Translation Velocity','FontSize',titlesize,'Interpreter','latex')

% angular omega double dot
% subplot(2,2,2)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\omega}}$ (rad/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Angular Acceleration','FontSize',titlesize,'Interpreter','latex')

% linear x double dot
% subplot(2,2,4)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\ddot{x}}$ (m/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Forward Acceleration','FontSize',titlesize,'Interpreter','latex')

% linear y double dot
% subplot(2,2,4)
nexttile(tcl)
hold on
for i = 1:1:num_paths
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.xdottable.Time,states.base.xdottable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.ydottable.Time,states.base.ydottable.Var1(:,i),'LineWidth',linewidth)
end
% legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\ddot{y}}$ (m/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Translation Acceleration','FontSize',titlesize,'Interpreter','latex')

hL = legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave');
hL.Layout.Tile = 'south';

if save
    pause(5)
    export_fig base.png -transparent -native
    pause(5)
    close
end