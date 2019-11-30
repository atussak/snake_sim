function proj_point = get_point_on_path(sim_point)

    x_sim = sim_point(1);
    y_sim = sim_point(2);
    
    
    % The curve is expressed with different functions in the areas
    % 0-3, 3-5, 5-7 and 7-inf
    
    syms x
    curve = {@(x) 0, @(x) sqrt(4 - (x-2.2)^2) - 2, @(x) -sqrt(4 - (x-6.2)^2) - 2, @(x) -4};
    section_partition = [0, 2.2, 4.2, 6.2]; % In x-direction
    sections = 4;
    % Find out in which areas we should 'search' for a good projection
    % point
    
    if x_sim <= section_partition(2)
        section = 1;
    elseif x_sim <= section_partition(3)
        section = 2;
    elseif x_sim <= section_partition(4)
        if y_sim < -2
            section = 3;
        else
            section = 2;
        end
    else
        section = 4;
    end
    
    shortest_dist = inf;
    step_length = 0.05;
    x_proj = 0;
    y_proj = 0;
    
    for sec = section:section%+1
        if sec <= sections %+ 1
            x = section_partition(sec);
            while x <= section_partition(sec+1)
                y = curve{sec}(x);
                if imag(y) ~= 0
                   disp('oh no not again')
                   sec
                   x
                end
                curr_point = [x y];
                diff = sim_point - curr_point;
                dist = sqrt(diff*diff');
                if dist < shortest_dist
                    shortest_dist = dist;
                    x_proj = x;
                    y_proj = y;
                end
                x = x + step_length;
            end
        end
    end
   
    proj_point = [x_proj y_proj];
end









