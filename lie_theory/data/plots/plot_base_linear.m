figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(1,2);

states.base.x = squeeze(xdot(3,:,:));
states.base.xtable = smoothdata(timetable(time',states.base.x));
states.base.xdot = [zeros([1,num_paths]);diff(states.base.xtable.Var1,1,1)];

states.base.xdottable = smoothdata(timetable(time',states.base.xdot/parameters.dt));

% linear x dot
% subplot(1,2,1)
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
title('Base Linear Velocity','FontSize',titlesize,'Interpreter','latex')

% linear x double dot
% subplot(1,2,2)
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
title('Base Linear Acceleration','FontSize',titlesize,'Interpreter','latex')

hL = legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave');
hL.Layout.Tile = 'East';

if save
    pause(5)
    export_fig base_linear.png -transparent -native
    pause(5)
    close
end