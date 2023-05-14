function plotter
    close all;
    global data
    sz = size(data);
    time_cur = data(:,1);
    position_x_cur = data(:,2);
    position_y_cur = data(:,3);
    position_z_cur = data(:,4);
    position_x_ref = data(:,5);
    position_y_ref = data(:,6);
    position_z_ref = data(:,7);   
    quaternion_cur = data(:,8:11);
    quaternion_des = data(:,12:15); 
    tic = 10;
    i = -18:3:18;
    y_ticks_array = [tic*i];
%_______________________________position figure________________________________________%
figure(1)
    grid on;      
    hold on;
    f1_index = 1:sz(1);
        plot(time_cur(f1_index),position_x_ref(f1_index),'color','r','LineWidth',4,'LineStyle','--');
        plot(time_cur(f1_index),position_y_ref(f1_index),'color','g','LineWidth',4,'LineStyle',':');
        plot(time_cur(f1_index),position_z_ref(f1_index),'color','b','LineWidth',4,'LineStyle','-.');
        plot(time_cur(f1_index),position_x_cur(f1_index),'color','c','LineWidth',3.5);
        plot(time_cur(f1_index),position_y_cur(f1_index),'color','m','LineWidth',3.5);
        plot(time_cur(f1_index),position_z_cur(f1_index),'color',[1 0.6940 0.1250],'LineWidth',3.5);
        xlim([10 50]); %time limit
        xlabel('time(sec)','FontSize',40);
        ylabel('position(m)','FontSize',40);
        set(gca,'linewidth',2,'fontsize',40,'fontname','Times');
        legend('x_{d}','y_{d}','z_{d}','x','y','z','FontSize',40);
        title('Position','FontSize',60);


%_______________________________attatude figure________________________________________%
figure(2)
    grid on;      
    hold on;
    f2_index = 1:sz(1);
        ypr_current(f2_index,1:3) = quat2eul(quaternion_cur(f2_index,1:4),'ZYX');   
        ypr_desired(f2_index,1:3) = quat2eul(quaternion_des(f2_index,1:4),'ZYX');
        ypr_current(f2_index,1:3) = rad2deg(ypr_current(f2_index,1:3));
        ypr_desired(f2_index,1:3) = rad2deg(ypr_desired(f2_index,1:3));

        plot(time_cur(f2_index),ypr_desired(f2_index,1),'color','r','LineWidth',4,'LineStyle','--');
        plot(time_cur(f2_index),ypr_desired(f2_index,2),'color','g','LineWidth',4,'LineStyle',':');
        plot(time_cur(f2_index),ypr_desired(f2_index,3),'color','b','LineWidth',4,'LineStyle','-.');
        plot(time_cur(f2_index),ypr_current(f2_index,1),'color','c','LineWidth',4);
        plot(time_cur(f2_index),ypr_current(f2_index,2),'color','m','LineWidth',3.5);
        plot(time_cur(f2_index),ypr_current(f2_index,3),'color',[1 0.6940 0.1250],'LineWidth',3.5);
        yticks(y_ticks_array);
        ylim([-180 180]);
        xlim([10 50]); %time limit
        xlabel('time(sec)','FontSize',40);
        ylabel('degree','FontSize',40);
        set(gca,'linewidth',2,'fontsize',40,'fontname','Times');
        legend('\psi_{d}','\theta_{d}','\phi_{d}','\psi','\theta','\phi','FontSize',40);
        title('Attitude','FontSize',60);
    
%_______________________________flight animate figure________________________________________%
figure(3)
    grid on;
    hold on;
    scatter3(position_x_ref, position_y_ref, position_z_ref,1,'MarkerEdgeColor','r','MarkerFaceColor','r');
    
    flight_path = animatedline('Color','b','LineWidth',2); 
    view(3);   % 3D view 

    for index = 2000:10000
        % flight path :
        addpoints(flight_path,position_x_cur(index),position_y_cur(index),position_z_cur(index));

        % flight attitude :
        if  mod(index,200) == 1   
        rotation_mat = quat2rotm(quaternion_cur(index,1:4));
        bx_axis = (rotation_mat*[0.5 0 0]')';
        by_axis = (rotation_mat*[0 0.5 0]')';
        bz_axis = (rotation_mat*[0 0 0.5]')';
        scatter3(position_x_cur(index),position_y_cur(index),position_z_cur(index),10,'MarkerEdgeColor','k','MarkerFaceColor','k');
        cur_p_txt = int2str(ceil(index/200)-1);
        text(position_x_cur(index),position_y_cur(index),position_z_cur(index),cur_p_txt,"FontSize",15);
%         text(position_x_ref(index), position_y_ref(index), position_z_ref(index),cur_p_txt,"FontSize",10);
        quiver3(position_x_cur(index),position_y_cur(index),position_z_cur(index),bx_axis(1),bx_axis(2),bx_axis(3),'Color','r');
        quiver3(position_x_cur(index),position_y_cur(index),position_z_cur(index),by_axis(1),by_axis(2),by_axis(3),'Color','g');
        quiver3(position_x_cur(index),position_y_cur(index),position_z_cur(index),bz_axis(1),bz_axis(2),bz_axis(3),'Color','b');
        end

        drawnow
        
        axis equal;
        xlabel('x(m)','FontSize',15);
        ylabel('y(m)','FontSize',15);
        zlabel('z(m)','FontSize',15);
        ylim([-1 5]);
        xlim([11 17]); %time limit
        title('flight path','FontSize',20);
    end 
end
