function [num_curves, curve_data, data_size] = get_curve_data(ox, oy, s)
    
    num_curves = 2;
    
    x_curve_1 = 2.2:0.01:4.2;
    y_curve_1 = zeros(1,length(x_curve_1));
    for i = 1:length(x_curve_1)
        y_curve_1(i) = (sqrt(4 - (x_curve_1(i)-2.2)^2) - 2-oy)*s;
    end
    x_curve_1 = (x_curve_1 - ox)*s;

    x_curve_2 = 4.2:0.01:6.2;
    y_curve_2 = zeros(1,length(x_curve_2));
    for i = 1:length(x_curve_2)
        y_curve_2(i) = (-sqrt(4 - (x_curve_2(i)-6.2)^2) - 2-oy)*s;
    end
    x_curve_2 = (x_curve_2 - ox)*s;  

    curve_data = [x_curve_1 y_curve_1;
                  x_curve_2 y_curve_2];
    
    data_size = length(x_curve_1);
end