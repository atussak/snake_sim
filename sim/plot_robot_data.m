function plot_robot_data(q, q_dot, q_dot_dot, pos, tauc, t)

    global n
    
    plot_q    = true;
    plot_qd   = false;
    plot_qdd  = false;
    plot_pos  = true;
    plot_tauc = false;
    
    q_legends   = '';
    qd_legends  = '';
    qdd_legends = '';
    tau_legends = '';
    
    % End effector position
    if plot_pos
        figure
        grid on
        pos(:,end) = [];
        plot(pos(1,:), pos(2,:))
        title('Head position')
        xlabel('x')
        ylabel('y')
    end
    
    % Joint angles
    if plot_q
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, q(i,:));
            legend_name = strcat('q', int2str(i));
            q_legends = [q_legends; legend_name];
        end
        legend(q_legends)
        title('Joint angles')
    end
    
    % Joint velocities
    if plot_qd
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, q_dot(i,:));
            legend_name = strcat('qd', int2str(i));
            qd_legends = [qd_legends; legend_name];
        end
        legend(qd_legends)
        title('Joint velocities')
    end
    
    if plot_qdd
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, q_dot_dot(i,:));
            legend_name = strcat('qdd', int2str(i));
            qdd_legends = [qdd_legends; legend_name];
        end
        legend(qdd_legends)
        title('Joint accelerations')
    end

    % Constraint torque
    if plot_tauc
        figure
        hold on
        grid on
        for i = 1:n+2
            plot(t, tauc(i,:));
            legend_name = strcat('tauc', int2str(i));
            tau_legends = [tau_legends; legend_name];
        end
        legend(tau_legends)
        title('Torque from constraint')
    end    




%     persistent p_q
%     global start
%     
%     % Plot joint angles
%     plot(q(:,1:k), t(1:k))
% 
%     
%     if start
%         figure('Resize','on','NumberTitle','on');
%         hold on;
%         % make plots for links
%         p_q = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
%         
%         hold off;
%         axis equal; axis off;
%         set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
%     else        
%         set(p_q,'xdata',[x0     c1(1)],'ydata',[y0 c1(2)]);
%         drawnow;
%     end

end