function plot_robot_data(q, t)
    persistent p_q
    global start
    
    % Plot joint angles
    plot(q(:,1:k), t(1:k))

    
    if start
        figure('Resize','on','NumberTitle','on');
        hold on;
        % make plots for links
        p_q = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
        
        hold off;
        axis equal; axis off;
        set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
    else        
        set(p_q,'xdata',[x0     c1(1)],'ydata',[y0 c1(2)]);
        drawnow;
    end

end