
% --------------------------------------------------------
% This function takes care of the visual simulation of the
% 2D snake robot as well as the obstacles ant path.
% For every time it is called, it deletes old drawings of
% links and replaces them with updated ones.
% The obstacles and the world frame illustration will just
% be drawn once and remain in this position.
% --------------------------------------------------------


function visualize(pos, x0, y0, proj_points, t)

    persistent p pp
    global start obstacle_coords num_obstacles n obstacle_radius
    global curve section_partition num_sections

    % Configuration for size and position of snake robot in window
    % Scaling factor
    s = 0.4;
    % Offset to move the origin of the coordinate frame
    ox = 1.7;
    oy = -0.5;
    
    if start
        start = false;

        figure('Resize','on','NumberTitle','on');
        
        hold on;
        
        %% Initialize plots for links
        for i = 1:n
            % Alternating colors for the links
            if mod(i,2) == 0
                p(i) = plot([0 0],[0 0],'-c','LineWidth',3,'Erasemode','xor');
            else
                p(i) = plot([0 0],[0 0],'-b','LineWidth',3,'Erasemode','xor');
            end
        end
        
        %% Initialize plots for projections
        for i = 1:n+1
            if i == 1 || i == n+1
                pp(i) = plot([0 0],[0 0],'--', 'color', [0.75 0.75 0.75],'LineWidth',1,'Erasemode','xor');
            else
                pp(i) = plot([0 0],[0 0],'--', 'color', 'k','LineWidth',1,'Erasemode','xor');    
            end
        end
        %% Draw base frame
        a = 1;
        ocx = ox*s*10;
        ocy = oy*s*10;
        plot([0-ocx 1-ocx]*0.1*a,[0-ocy 0-ocy]*0.1*a,'r'); % base frame (x-axis)
        plot([0-ocx 0-ocx]*0.1*a,[0-ocy 1-ocy]*0.1*a,'g'); % base frame (y-axis)
        
        %% Draw obstacles
        th = 0:pi/50:2*pi;
        r = obstacle_radius;
        for i = 1:num_obstacles
           x_obs = obstacle_coords(i,1)-ox;
           y_obs = obstacle_coords(i,2)-oy;
           xunit = r * cos(th) + x_obs;
           yunit = r * sin(th) + y_obs;
           plot(xunit*s, yunit*s, 'r');
        end
        
        
        %% Draw desired trajectory
        orange = [1 0.498 0.3137];
        syms x
        for i = 1:num_sections
            if diff(curve{i}, x) == 0 % this is a line
                start_x = section_partition(i);
                if i == num_sections
                    end_x = start_x + 100;
                else
                    end_x = section_partition(i+1);
                end
                start_y = curve{i}(0);
                end_y = start_y;
                plot(([start_x,end_x]-[ox,ox])*s, ([start_y,end_y]-[oy,oy])*s, 'color', orange)
            end
        end
        [num_curves, curve_data, data_size] = get_curve_data(ox, oy, s);
        for i = 1:num_curves
            plot(curve_data(i,1:data_size), curve_data(i,data_size+1:data_size*2), 'color', orange);
        end

        %% Draw-settings
        hold off;
        axis equal; axis off;
        set(gca,'Drawmode','Fast','NextPlot','ReplaceChildren');
        axis([-a a -a a]);

        
    else
        
        %% Re-draw links
      
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
        
        %% Re-draw projection lines
      
        d = zeros(n+1,2);
        for i = 1:n+1
           d(i,:) = (proj_points(i,:)-[ox,oy])*s;
        end
      
        set(pp(n+1),'xdata',[x0 d(n+1,1)],'ydata',[y0 d(n+1,2)]);
        for i = 1:n
            set(pp(i),'xdata',[c(i,1) d(i,1)],'ydata',[c(i,2) d(i,2)]);
        end
        
        %%
        drawnow;
end