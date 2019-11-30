function q_ref = get_path_reference_angle2(prev_point, point, proj_point, q_sim)

    a = proj_point - point;
    a = sqrt(a*a');

    b = prev_point - point;
    b = sqrt(b*b');
    
    c = prev_point - proj_point;
    c = sqrt(c*c');
    
    q_diff = acos((b^2 + c^2 - a^2)/(2*b*c));

    % Increment or decrement the reference?
    
%     x = point(1);
%     y = point(2);
%     proj_x = proj_point(1);
%     proj_y = proj_point(2);
%     
%     if abs(proj_y - y) > abs(proj_x - y)
%         if y > proj_y
%             q_ref = q_sim - q_diff;
%         else
%             q_ref = q_sim + q_diff; 
%         end
%     else
%         if x > proj_x
%             q_ref = q_sim - q_diff;
%         else
%             q_ref = q_sim + q_diff; 
%         end
%     end
    
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
        end
    end
    
    
    
    
    
    
end