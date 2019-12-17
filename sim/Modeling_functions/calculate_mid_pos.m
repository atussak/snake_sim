function mid_pos = calculate_mid_pos(q)

    global n l

    mid_pos = zeros(n,2);
    
    for link = 1:n
       x = q(n+1);
       y = q(n+2);
       for j = 1:link
          q_temp = 0;
          for k = 1:j % For absolute angles
             q_temp = q_temp + q(k); 
          end
          if j == link % To get to the center of mass (middle) of the link
              x = x + (l/2)*cos(q_temp);
              y = y + (l/2)*sin(q_temp);
          else
              x = x + l*cos(q_temp);
              y = y + l*sin(q_temp);
          end
       end
       mid_pos(link,:) = [x y];
    end

end