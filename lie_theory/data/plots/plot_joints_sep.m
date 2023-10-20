states.arm.q = xdot(dof.base+1:end,:,:);
states.arm.qtable = smoothdata(timetable(time',pagetranspose(states.arm.q)));
states.arm.qdot = [zeros([1,6,num_paths]);diff(states.arm.qtable.Var1,1,1)];
states.arm.qdottable = smoothdata(timetable(time',(states.arm.qdot/parameters.dt)));

% vertical helix
figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,2,1)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')

subplot(1,2,2)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

if save
    pause(5)
    export_fig arm_states_vh.png -transparent -native
    pause(5)
    close
end

% sine wave
figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,2,1)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,2)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Sine Wave','FontSize',titlesize,'Interpreter','latex')

subplot(1,2,2)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,2)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

if save
    pause(5)
    export_fig arm_states_sw.png -transparent -native
    pause(5)
    close
end

% horizontal helix
figure('units','normalized','outerposition',[0 0 1 1])

subplot(1,2,1)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,3)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')

subplot(1,2,2)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,3)','LineWidth',linewidth)
legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

if save
    pause(5)
    export_fig arm_states_hh.png -transparent -native
    pause(5)
    close
end

% % % spiral
% % subplot(2,4,4)
% % hold on
% % plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,4)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square
% % title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')
% % 
% % subplot(2,4,8)
% % hold on
% % plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,4)','LineWidth',linewidth)
% % legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
% % grid minor
% % set(gca,'fontsize',fontsize)
% % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
% % ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
% % xlim(seconds([0,20]))
% % axis square

