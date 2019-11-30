function [phi, point_diff, angle_diff, curr_left] = get_path_reference_angle(sim_point, proj_point, sim_phi, prev_joint_diff, prev_point_diff, prev_left)
    
%     global l

    sim_x = sim_point(1);
%     sim_y = sim_point(2);
    proj_x = proj_point(1);
%     proj_y = proj_point(2);
    
    curr_left = true;
    if proj_x < sim_x
        curr_left = false;
    end
    
    point_diff = sim_point - proj_point;
    abs_point_diff = sqrt(point_diff*point_diff');
    
    if prev_left && curr_left
        angle_diff = asin(abs_point_diff - prev_point_diff) + prev_joint_diff;
    elseif prev_left && ~curr_left
        angle_diff = -asin(abs_point_diff - prev_point_diff);
    elseif ~prev_left && curr_left
        angle_diff = asin(abs_point_diff - prev_point_diff);
    else
        angle_diff = -(asin(abs_point_diff - prev_point_diff) + prev_joint_diff);        
    end
    phi = sim_phi + angle_diff;
%     if prev_joint_diff < 0
%         part_point_diff = abs_point_diff/(l + prev_joint_diff);
%         phi_diff = asin(part_point_diff);
%     else
%         part_point_diff = abs_point_diff - prev_joint_diff;
%         phi_diff = asin(part_point_diff) + asin(prev_joint_diff);
%     end
%     
%     
%     if sim_y > proj_y
%         phi = sim_phi - phi_diff;
%     else
%         phi = sim_phi + phi_diff;
%     end

end