function plot_robot_data(q, q_dot, q_dot_dot, q_ref, pos, tau, error, energy, momentum, t)

    global n
    
    plot_q          = 0;
    plot_qd         = 0;
    plot_qdd        = 0;
    plot_pos        = 0;
    plot_tauc       = 0;
    plot_q_ref      = 0;
    plot_error      = 0;
    plot_energy     = 0;
    plot_momentum   = 0;
    
    q_legends       = '';
    qd_legends      = '';
    qdd_legends     = '';
    tau_legends     = '';
    q_ref_legends   = '';
    error_legends   = '';
    
    fontsize_label = 10;
    fontsize_legend = 9;
    fontsize_title  = 10;
    textwidth = 12.12364; % cm
    textheight = 16.3757; %cm
    green = [0.24 0.7 0.44];
    orange = [1 0.65 0];
    purple = [0.5 0 0.5];
    
    % End effector position
    
    if plot_pos
        figure
%         grid on
        hold on
        pos(:,end) = [];
        plot(pos(1,:), pos(2,:))
        axis([3 5 -2 0])
        title('Head  position')
        xlabel('x')
        ylabel('y')
        % Draw desired trajectory
        orange = [1 0.498 0.3137];
        plot([0,2.2], [0,0], 'color', orange)
        plot([6.2,10], [-4,-4], 'color', orange) 
        [num_curves, curve_data, data_size] = get_curve_data(0, 0, 1);
        for i = 1:num_curves
            plot(curve_data(i,1:data_size), curve_data(i,data_size+1:data_size*2), 'color', orange);
        end
        
%         figure
%         hold on
%         % y position
%         subplot(1,2,1)
%         grid on
%         plot(t(1:length(t)-1), pos(2,:));
%         legend('y')
%         title('Head y position')
%         % x position
%         subplot(1,2,2)
%         grid on
%         plot(t(1:length(t)-1), pos(1,:));
%         legend('y')
%         title('Head x position')
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
    
    % Joint references
    if plot_q_ref
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, q_ref(i,:));
            legend_name = strcat('q_{ref}', int2str(i));
            q_ref_legends = [q_ref_legends; legend_name];
        end
        legend(q_ref_legends)
        title('Joint references')
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

    % Joint torque
    if plot_tauc
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, tau(i,:));
            legend_name = strcat('tauc', int2str(i));
            tau_legends = [tau_legends; legend_name];
        end
        legend(tau_legends)
        title('Joint torques')
    end    

    % Joint error
    if plot_error
        figure
        hold on
        grid on
        for i = 1:n
            plot(t, error(i,:));
            legend_name = strcat('err', int2str(i));
            error_legends = [error_legends; legend_name];
        end
        legend(error_legends)
        title('Joint angle error')
    end
    
    if plot_energy
        energy(1) = 0;
        fig = figure('PaperPositionMode', 'Auto', 'Units', 'centimeters', 'Position', [0 0 textwidth 0.90*textheight]);
        clf
        
        subplot(3,1,1);
        hold on
        %grid on
        energy(3000) = energy(2999);
        plot(t, energy, 'LineWidth',1);
        legend('$K$', 'Interpreter', 'latex')
        title('Energy', 'Interpreter', 'latex')
        xlabel('$[s]$','Interpreter','latex')
        ylabel('$[J]$','Interpreter','latex')
        hold off
        
        subplot(3,1,2);
        momentum(1,1) = 0;
        %momentum(:,3000) = momentum(:,2999);
        hold on
        plot(t, momentum(1,:), '-', 'color', orange, 'LineWidth',1);
        plot(t, momentum(2,:), '--', 'color', purple, 'LineWidth',1);
        legend('$p_x$', '$p_y$', 'Interpreter', 'latex')
        title('Momentum', 'Interpreter','latex', 'FontSize', fontsize_title)
        xlabel('$[s]$','Interpreter','latex')
        ylabel('$[kg\cdot m/s]$','Interpreter','latex')
        hold off
        
        subplot(3,1,3); 
        hold on
        plot(t, tau(2,:), '-', 'color', green, 'LineWidth',1);
        plot(t, tau(3,:), '--', 'color', orange, 'LineWidth',1);
        plot(t, tau(4,:), '--', 'color', purple, 'LineWidth',1);
        legend('$\tau_2$', '$\tau_3$', '$\tau_4$', 'Interpreter','latex','FontSize', fontsize_legend)
        title('Joint motor torques', 'Interpreter','latex', 'FontSize', fontsize_title)
        xlabel('$[s]$','Interpreter','latex')
        ylabel('$[Nm]$','Interpreter','latex')
        hold off
        
        
%         set(gca,...
%         'FontUnits','points',...
%         'FontWeight','normal',...
%         'FontSize',fontsize_label)
%         
%         print_figure(gcf, 'energy-momentum')
    end
    
    if plot_momentum
        fig = figure('PaperPositionMode', 'Auto', 'Units', 'centimeters', 'Position', [0 0 textwidth 0.50 * textheight]);
        clf
        momentum(1,1) = 0;
        %momentum(:,3000) = momentum(:,2999);
        hold on
        plot(t, momentum(1,:), '-', 'color', orange, 'LineWidth',1);
        plot(t, momentum(2,:), '--', 'color', purple, 'LineWidth',1);
        legend('$p_x$', '$p_y$', 'Interpreter', 'latex')
        title('Momentum of robot', 'Interpreter','latex', 'FontSize', fontsize_title)
        xlabel('$[s]$','Interpreter','latex')
        ylabel('$[kg\cdot m/s]$','Interpreter','latex')
        hold off
        
        
%         set(gca,...
%         'FontUnits','points',...
%         'FontWeight','normal',...
%         'FontSize',fontsize_label)
%         
%         print_figure(gcf, 'momentum')
    end
    

end