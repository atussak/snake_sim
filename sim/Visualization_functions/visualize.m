
% --------------------------------------------------------
% This function takes care of the visual simulation of the
% 2D snake robot as well as the obstacles.
% For every time it is called, it deletes old drawings of
% links and replaces them with updated ones.
% The obstacles and the world frame illustration will just
% be drawn once and remain in this position.
% --------------------------------------------------------


function visualize(pos, x0, y0)

    persistent p
    global start obstacle_coords num_obstacles n obstacle_radius

    % Scaling factor
    s = 0.3;
    % Offset to move the origin of the coordinate frame
    ox = 3.3;
    oy = -2;
    
    if start
        start = false;
        
        figure('Resize','on','NumberTitle','on');
        hold on;
        
        % Initialize plots for links
        for i = 1:n
            % Alternating colors for the links
            if mod(i,2) == 0
                p(i) = plot([0 0],[0 0],'-c','LineWidth',3,'Erasemode','xor');
            else
                p(i) = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
            end
        end
        
        % Draw base frame
        a = 1;
        ocx = ox*s*10;
        ocy = oy*s*10;
        plot([0-ocx 1-ocx]*0.1*a,[0-ocy 0-ocy]*0.1*a,'r'); % base frame (x-axis)
        plot([0-ocx 0-ocx]*0.1*a,[0-ocy 1-ocy]*0.1*a,'g'); % base frame (y-axis)
        
        % Draw obstacles
        th = 0:pi/50:2*pi;
        r = obstacle_radius;
        for i = 1:num_obstacles
           x_obs = obstacle_coords(i,1)-ox;
           y_obs = obstacle_coords(i,2)-oy;
           xunit = r * cos(th) + x_obs;
           yunit = r * sin(th) + y_obs;
           plot(xunit*s, yunit*s, 'r');
        end
        
        
        % Draw desired trajectory
        orange = [1 0.498 0.3137];
        plot(([0,2.2]-[ox,ox])*s, ([0,0]-[oy,oy])*s, 'color', orange)
        plot(([6.2,10]-[ox,ox])*s, ([-4,-4]-[oy,oy])*s, 'color', orange) 
        [num_curves, curve_data, data_size] = get_curve_data(ox, oy, s);
        for i = 1:num_curves
            plot(curve_data(i,1:data_size), curve_data(i,data_size+1:data_size*2), 'color', orange);
        end

        % Draw settings
        hold off;
        axis equal; axis off;
        set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
        axis([-a a -a a]);

        
    else
        
        % Re-draw links
      
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