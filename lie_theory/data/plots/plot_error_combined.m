h_results = load("husky_ur5e_holo_results.mat");
nh_results = load("husky_ur5e_results.mat");

num_paths = nh_results.num_paths;
time = nh_results.time;
pose_goals_helix = nh_results.pose_goals_helix;
pose_goals_sine = nh_results.pose_goals_sine;
pose_goals_horizontal = nh_results.pose_goals_horizontal;
linewidth = nh_results.linewidth;
fontsize = nh_results.fontsize;
labelsize = nh_results.labelsize;
titlesize = nh_results.titlesize;
save = nh_results.save;

linear_axis = 0.000005;
angular_axis = 0.000005;

figure('units','normalized','outerposition',[0 0 1 1])
tcl = tiledlayout(2,num_paths);

for iConfig = 1:2
    if iConfig == 1
        tool_pose = nh_results.tool_pose;
    elseif iConfig == 2
        tool_pose = h_results.tool_pose;
    else
        tool_pose = nh_results.tool_pose;
    end
    %% vertical helix
    pose_goal = tform(pose_goals_helix);
    error_plot.x = squeeze(tool_pose(1,4,:,1)-pose_goal(1,4,1:end-1));
    error_plot.y = squeeze(tool_pose(2,4,:,1)-pose_goal(2,4,1:end-1));
    error_plot.z = squeeze(tool_pose(3,4,:,1)-pose_goal(3,4,1:end-1));
    for m = 1:1:length(tool_pose)
        vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,1));
        vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
        error_plot.rx(m) = vec1(1)-vec2(1);
        error_plot.ry(m) = vec1(2)-vec2(2);
        error_plot.rz(m) = vec1(3)-vec2(3);
    end
    
    error_plot.xtable = smoothdata(timetable(time',error_plot.x));
    error_plot.ytable = smoothdata(timetable(time',error_plot.y));
    error_plot.ztable = smoothdata(timetable(time',error_plot.z));
    error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
    error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
    error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
    
    residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
    rmse_pos(1) = sqrt(mean(residuals.^2,'all'));
    residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
    rmse_rot(1) = sqrt(mean(residuals.^2,'all'));
    
    % subplot(2,num_paths,1)
    nexttile(tcl,1)
    hold on
    plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
    plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
    plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
    % legend('x','y','z')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Position Error (m)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    title('Vertical Helix','FontSize',titlesize,'Interpreter','latex')
    ylim([-linear_axis,linear_axis])
    
    % subplot(2,num_paths,num_paths+1)
    nexttile(tcl,4)
    hold on
    plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
    plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
    plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
    % legend('roll','pitch','yaw')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Orientation Error (rad)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    ylim([-angular_axis,angular_axis])
    
    %% sine wave
    pose_goal = tform(pose_goals_sine);
    error_plot.x = squeeze(tool_pose(1,4,:,2)-pose_goal(1,4,1:end-1));
    error_plot.y = squeeze(tool_pose(2,4,:,2)-pose_goal(2,4,1:end-1));
    error_plot.z = squeeze(tool_pose(3,4,:,2)-pose_goal(3,4,1:end-1));
    for m = 1:1:length(tool_pose)
        vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,2));
        vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
        error_plot.rx(m) = vec1(1)-vec2(1);
        error_plot.ry(m) = vec1(2)-vec2(2);
        error_plot.rz(m) = vec1(3)-vec2(3);
    end
    
    error_plot.xtable = smoothdata(timetable(time',error_plot.x));
    error_plot.ytable = smoothdata(timetable(time',error_plot.y));
    error_plot.ztable = smoothdata(timetable(time',error_plot.z));
    error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
    error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
    error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
    
    residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
    rmse_pos(2) = sqrt(mean(residuals.^2,'all'));
    residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
    rmse_rot(2) = sqrt(mean(residuals.^2,'all'));
    
    % subplot(2,num_paths,2)
    nexttile(tcl,2)
    hold on
    plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
    plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
    plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
    % legend('x','y','z')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Position Error (m)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    title('Sine Wave','FontSize',titlesize,'Interpreter','latex')
    ylim([-linear_axis,linear_axis])
    
    % subplot(2,num_paths,num_paths+2)
    nexttile(tcl,5)
    hold on
    plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
    plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
    plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
    % legend('roll','pitch','yaw')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Orientation Error (rad)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    ylim([-angular_axis,angular_axis])
    
    %% horizontal helix
    pose_goal = tform(pose_goals_horizontal);
    error_plot.x = squeeze(tool_pose(1,4,:,3)-pose_goal(1,4,1:end-1));
    error_plot.y = squeeze(tool_pose(2,4,:,3)-pose_goal(2,4,1:end-1));
    error_plot.z = squeeze(tool_pose(3,4,:,3)-pose_goal(3,4,1:end-1));
    for m = 1:1:length(tool_pose)
        vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,3));
        vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
        error_plot.rx(m) = vec1(1)-vec2(1);
        error_plot.ry(m) = vec1(2)-vec2(2);
        error_plot.rz(m) = vec1(3)-vec2(3);
    end
    
    error_plot.xtable = smoothdata(timetable(time',error_plot.x));
    error_plot.ytable = smoothdata(timetable(time',error_plot.y));
    error_plot.ztable = smoothdata(timetable(time',error_plot.z));
    error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
    error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
    error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
    
    residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
    rmse_pos(3) = sqrt(mean(residuals.^2,'all'));
    residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
    rmse_rot(3) = sqrt(mean(residuals.^2,'all'));
    
    % subplot(2,num_paths,3)
    nexttile(tcl,3)
    hold on
    plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
    plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
    plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
    legend('x_{non-holonomic}','y_{non-holonomic}','z_{non-holonomic}','x_{holonomic}','y_{holonomic}','z_{holonomic}','Location','eastoutside')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Position Error (m)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    title('Horizontal Helix','FontSize',titlesize,'Interpreter','latex')
    ylim([-linear_axis,linear_axis])
    
    % subplot(2,num_paths,num_paths+3)
    nexttile(tcl,6)
    hold on
    plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
    plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
    plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
    legend('roll_{non-holonomic}','pitch_{non-holonomic}','yaw_{non-holonomic}','roll_{holonomic}','pitch_{holonomic}','yaw_{holonomic}','Location','eastoutside')
    grid minor
    set(gca,'fontsize',fontsize)
    xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    ylabel('\textbf{Orientation Error (rad)}','FontSize',labelsize,'Interpreter','latex')
    xlim(seconds([0,20]))
    axis square
    ylim([-angular_axis,angular_axis])
    
    % % % spiral
    % % pose_goal = tform(pose_goals_spiral);
    % % error_plot.x = squeeze(tool_pose(1,4,:,3)-pose_goal(1,4,1:end-1));
    % % error_plot.y = squeeze(tool_pose(2,4,:,3)-pose_goal(2,4,1:end-1));
    % % error_plot.z = squeeze(tool_pose(3,4,:,3)-pose_goal(3,4,1:end-1));
    % % for m = 1:1:length(tool_pose)
    % %     vec1 = rotmat2vec3d(tool_pose(1:3,1:3,m,3));
    % %     vec2 = rotmat2vec3d(pose_goal(1:3,1:3,m));
    % %     error_plot.rx(m) = vec1(1)-vec2(1);
    % %     error_plot.ry(m) = vec1(2)-vec2(2);
    % %     error_plot.rz(m) = vec1(3)-vec2(3);
    % % end
    % % 
    % % error_plot.xtable = smoothdata(timetable(time',error_plot.x));
    % % error_plot.ytable = smoothdata(timetable(time',error_plot.y));
    % % error_plot.ztable = smoothdata(timetable(time',error_plot.z));
    % % error_plot.rxtable = smoothdata(timetable(time',error_plot.rx'));
    % % error_plot.rytable = smoothdata(timetable(time',error_plot.ry'));
    % % error_plot.rztable = smoothdata(timetable(time',error_plot.rz'));
    % % 
    % % residuals = [error_plot.xtable.Var1,error_plot.ytable.Var1,error_plot.ztable.Var1];
    % % rmse_pos(4) = sqrt(mean(residuals.^2,'all'));
    % % residuals = [error_plot.rxtable.Var1,error_plot.rytable.Var1,error_plot.rztable.Var1];
    % % rmse_rot(4) = sqrt(mean(residuals.^2,'all'));
    % % 
    % % subplot(2,4,4)
    % % hold on
    % % plot(error_plot.xtable.Time,error_plot.xtable.Var1,'LineWidth',linewidth)
    % % plot(error_plot.ytable.Time,error_plot.ytable.Var1,'LineWidth',linewidth)
    % % plot(error_plot.ztable.Time,error_plot.ztable.Var1,'LineWidth',linewidth)
    % % legend('x','y','z')
    % % grid minor
    % % set(gca,'fontsize',fontsize)
    % % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    % % ylabel('\textbf{error (m)}','FontSize',labelsize,'Interpreter','latex')
    % % xlim(seconds([0,20]))
    % % axis square
    % % title('Spiral Wave','FontSize',titlesize,'Interpreter','latex')
    % % 
    % % subplot(2,4,8)
    % % hold on
    % % plot(error_plot.rxtable.Time,error_plot.rxtable.Var1,'LineWidth',linewidth)
    % % plot(error_plot.rytable.Time,error_plot.rytable.Var1,'LineWidth',linewidth)
    % % plot(error_plot.rztable.Time,error_plot.rztable.Var1,'LineWidth',linewidth)
    % % legend('roll','pitch','yaw')
    % % grid minor
    % % set(gca,'fontsize',fontsize)
    % % xlabel('\textbf{Time}','FontSize',labelsize,'Interpreter','latex')
    % % ylabel('\textbf{error (rad)}','FontSize',labelsize,'Interpreter','latex')
    % % xlim(seconds([0,20]))
    % % axis square
    
    % hL1 = legend('x','y','z');
    % hL1.Layout.Tile = 'East';
    % 
    % hL2 = legend('roll','pitch','yaw');
    % hL2.Layout.Tile = 'southeast';
end

if save
    pause(5)
    export_fig error.png -transparent -native
    pause(5)
    close
end