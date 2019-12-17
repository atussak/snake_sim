function q_ref = get_path_reference_angle(prev_point, point, proj_point, q_sim, linknum)

    a = proj_point - point;
    a = sqrt(a*a');
    
    if a < 0.001
        q_ref = q_sim;
        return
    end    
    
    b = prev_point - point;
    b = sqrt(b*b');
    
    c = prev_point - proj_point;
    c = sqrt(c*c');
    
    q_diff = acos((b^2 + c^2 - a^2)/(2*b*c));

    % Increment or decrement the reference?
    % --> try both and choose the one that gives the best result
    % Brute force, but robust, method
    
    q_diff_sgn = [-q_diff, q_diff];
    link_vec = point - prev_point;
    
    best_dist = inf;
    
    for i = 1:2
        phi = q_diff_sgn(i);
        R = [cos(phi) -sin(phi);
             sin(phi)  cos(phi)];
        test_link_vec = R*link_vec';
        
        test_point = test_link_vec' + prev_point;
        dist = test_point - proj_point;
        dist = sqrt(dist*dist');
        
        if dist < best_dist
            best_dist = dist;
            q_ref = q_sim + phi;
            if linknum == 1
               q_ref = phi;
            end
        end
    end
    
    
    
end