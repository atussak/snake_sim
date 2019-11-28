function phi = get_path_reference_angle(sim_point, proj_point, old_phi)
    sim_y = sim_point(2);
    proj_y = proj_point(2);
    
    point_diff = sim_point - proj_point;
    abs_point_diff = sqrt(point_diff*point_diff');
    phi_diff = asin(abs_point_diff);
    
    if sim_y > proj_y
        phi = old_phi - phi_diff;
    else
        phi = old_phi + phi_diff;
    end
    
end