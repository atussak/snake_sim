function y = get_point_on_path(x)

    if x <= 3
        y = 0;
    elseif x <= 5
        y = sqrt(4 - (x-3)^2) - 2;
    elseif x <= 7
        y = -sqrt(4 - (x-7)^2) - 2;
    else
        y = -4;
    end

end