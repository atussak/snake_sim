
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
    global start obstacle_coords num_obstacles n

    % Scaling factor
    s = 0.3;
    % Offset to move the origin of the coordinate frame
    ox = 2.5;
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
        plot([0-ocx 1-ocx]*0.1*a,[0-ocy 0-ocy]*0.1*a,'r');     % base frame (x-axis)
        plot([0-ocx 0-ocx]*0.1*a,[0-ocy 1-ocy]*0.1*a,'g');     % base frame (y-axis)
        
        % Draw obstacles
        for i = 1:num_obstacles
           x_obs = obstacle_coords(i,1)-ox;
           y_obs = obstacle_coords(i,2)-oy;
           plot(x_obs*s, y_obs*s, '^r', 'MarkerFaceColor','r') 
        end
        
        plot(([0,2.5]-[ox,ox])*s, ([0,0]-[oy,oy])*s, 'r')
        %plot(([7,10]-[ox,ox])*s, ([-4,-4]-[oy,oy])*s, 'r')
        
        x_curve = 2.5:0.01:4.5;
        y_curve = zeros(1,length(x_curve));
        for i = 1:length(x_curve)
            y_curve(i) = (sqrt(4 - (x_curve(i)-2.5)^2) - 2-oy)*s;
        end
        x_curve = (x_curve - ox)*s;
        
        plot(x_curve, y_curve, 'r');

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