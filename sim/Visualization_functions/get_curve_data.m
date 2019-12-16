function [num_curves, curve_data, data_size] = get_curve_data(ox, oy, s)
    
    global curve section_partition num_sections
    
    num_curves = 0;
    curve_data = [];
    syms x
    
    for sec = 1:num_sections
        if diff(curve{sec}, x) ~= 0 % this is not a line
            num_curves = num_curves + 1;
            x_curve = section_partition(sec):0.01:section_partition(sec+1);
            y_curve = zeros(1,length(x_curve));
            for i = 1:length(x_curve)
                y_curve(i) = (curve{sec}(x_curve(i))-oy)*s;
            end
            x_curve = (x_curve - ox)*s;

            curve_data = [curve_data;
                          x_curve y_curve];

            data_size = length(x_curve);
        end
    end
    
end