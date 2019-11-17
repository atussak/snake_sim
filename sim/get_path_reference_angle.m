function phi = get_path_reference_angle(x, t)

    x = x - 0.3

    if x <= 2
        phi = 0;
    elseif x <= 3
        phi = -pi/2;
    elseif x <= 3.5
        phi = 0;
    elseif x <= 3.8
        phi = pi/4;
    else
        phi = 0;
    end
    
end