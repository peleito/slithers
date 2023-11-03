figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(2,num_paths);

states.arm.q = xdot(dof.base+1:end,:,:);
states.arm.qtable = smoothdata(timetable(time',pagetranspose(states.arm.q)));
states.arm.qdot = [zeros([1,6,num_paths]);diff(states.arm.qtable.Var1,1,1)];
states.arm.qdottable = smoothdata(timetable(time',(states.arm.qdot/parameters.dt)));

% vertical helix
% subplot(2,num_paths,1)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,1)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')

% sine wave
% subplot(2,num_paths,2)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,2)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Sine Wave','FontSize',titlesize,'Interpreter','latex')

% horizontal helix
% subplot(2,num_paths,3)
nexttile(tcl)
hold on
plot(states.arm.qtable.Time,states.arm.qtable.Var1(:,:,3)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\theta}$ (rad)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square
title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')

% vertical helix
% subplot(2,num_paths,num_paths+1)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,1)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% sine wave
% subplot(2,num_paths,num_paths+2)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,2)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

% horizontal helix
% subplot(2,num_paths,num_paths+3)
nexttile(tcl)
hold on
plot(states.arm.qdottable.Time,states.arm.qdottable.Var1(:,:,3)','LineWidth',linewidth)
% legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3')
grid minor
set(gca,'fontsize',fontsize)
xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
ylabel('\textbf{$\mathbf{\dot{\theta}}$ (rad/sec)}','FontSize',labelsize,'Interpreter','latex')
xlim(seconds([0,20]))
axis square

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

hL = legend('Shoulder Pan','Shoulder Lift','Elbow','Wrist 1','Wrist 2','Wrist 3');
hL.Layout.Tile = 'East';

if save
    pause(5)
    export_fig arm_states.png -transparent -native
    pause(5)
    close
end