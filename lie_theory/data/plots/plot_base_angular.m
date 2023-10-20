figure('units','normalized','outerposition',[0 0 1 1])

states.base.omega = squeeze(xdot(1,:,:));
states.base.omegatable = smoothdata(timetable(time',states.base.omega));
states.base.omegadot = [zeros([1,num_paths]);diff(states.base.omegatable.Var1,1,1)];
states.base.omegadottable = smoothdata(timetable(time',states.base.omegadot/parameters.dt));

% angular omega dot
subplot(1,2,1)
hold on
for i = 1:1:num_paths
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.omegatable.Time,states.base.omegatable.Var1(:,i),'LineWidth',linewidth)
end
legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\omega}$ (rad/s)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Angular Velocity','FontSize',titlesize,'Interpreter','latex')

% angular omega double dot
subplot(1,2,2)
hold on
for i = 1:1:num_paths
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,1),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,2),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,3),'LineWidth',linewidth)
    % plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,4),'LineWidth',linewidth)
    plot(states.base.omegadottable.Time,states.base.omegadottable.Var1(:,i),'LineWidth',linewidth)
end
legend('Vertical Helix','Sine Wave','Horizontal Helix','Spiral Wave')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\omega}}$ (rad/s$^\mathbf{2}$)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Base Angular Acceleration','FontSize',titlesize,'Interpreter','latex')

if save
    pause(5)
    export_fig base_angular.png -transparent -native
    pause(5)
    close
end