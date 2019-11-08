function visualize(pos, x0, y0)

    persistent p
    global start obstacle_coords num_obstacles n

    % Scaling factor
    s = 0.5;
    % Offset to move the origin of the coordinate frame
    ox = 2;
    oy = 0.2;
    
    if start
        start = false;
        figure('Resize','on','NumberTitle','on');
        hold on;
        % make plots for links
        for i = 1:n
            % Alternating colors for the links
            if mod(i,2) == 0
                p(i) = plot([0 0],[0 0],'-c','LineWidth',3,'Erasemode','xor');
            else
                p(i) = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
            end
        end
        
        % base frame
        a = 1;
        ocx = ox*s*10;
        ocy = oy*s*10;
        plot([0-ocx 1-ocx]*0.1*a,[0-ocy 0-ocy]*0.1*a,'r');     % base frame (x-axis)
        plot([0-ocx 0-ocx]*0.1*a,[0-ocy 1-ocy]*0.1*a,'g');     % base frame (y-axis)
        
        % obstacles
        for i = 1:num_obstacles
           x_obs = obstacle_coords(i,1)-ox;
           y_obs = obstacle_coords(i,2)-oy;
           plot(x_obs*s, y_obs*s, '^r', 'MarkerFaceColor','r') 
        end
        
        hold off;
        axis equal; axis off;
        set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
        axis([-a a -a a]);
    else
        
        c = zeros(n,2);
        for i = 1:n
           c(i,:) = (pos(i,:)-[ox,oy])*s;
        end
        
        x0 = (x0-ox)*s;
        y0 = (y0-oy)*s;
        
        set(p(1),'xdata',[x0    c(1,1)],'ydata',[y0 c(1,2)]);
        for i = 2:n
            set(p(i),'xdata',[c(i-1,1) c(i,1)],'ydata',[c(i-1,2) c(i,2)]);
        end
        
        drawnow;
    end