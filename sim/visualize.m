function visualize(pos, x0, y0)

    persistent p1 p2 p3
    global start obstacle_coords num_obstacles

    if start
        start = false;
        figure('Resize','on','NumberTitle','on');
        hold on;
        % make plots for links
        p1 = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
        p2 = plot([0 0],[0 0],'-c','LineWidth',3,'Erasemode','xor');
        p3 = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
        
        % base frame
        a = 1;
        plot([0 1]*0.1*a,[0 0]*0.1*a,'r');     % base frame (x-axis)
        plot([0 0]*0.1*a,[0 1]*0.1*a,'g');     % base frame (y-axis)
        
        % obstacles
        for i = 1:num_obstacles
           x_obs = obstacle_coords(i,1);
           y_obs = obstacle_coords(i,2);
           plot(x_obs*0.3, y_obs*0.3, '^r', 'MarkerFaceColor','r') 
        end
        
        hold off;
        axis equal; axis off;
        set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
        axis([-a a -a a]);
    else
        
        c1 = pos(1,:)*0.3;
        c2 = pos(2,:)*0.3;
        c3 = pos(3,:)*0.3;
        
        x0 = x0*0.3;
        y0 = y0*0.3;
        
        set(p1,'xdata',[x0     c1(1)],'ydata',[y0 c1(2)]);
        set(p2,'xdata',[c1(1) c2(1)],'ydata',[c1(2) c2(2)]);
        set(p3,'xdata',[c2(1) c3(1)],'ydata',[c2(2) c3(2)]);
        
        drawnow;
    end