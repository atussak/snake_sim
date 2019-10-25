function tauc = calc_tauc(pos, q_sim, tau)

  global n l num_obstacles obstacle_coords Jc_func

  tauc = zeros(n, 1);

  % For every obstacle
  for i = 1:num_obstacles
    % Obstacle coordinate
    C = obstacle_coords(i,:);

    % For every link
    for j = 1:n
      % Link line segment A-B
      B = pos(j,:);
      A = [q_sim(n+1) q_sim(n+2)]; % Last joint in origin if first link
      if j ~= 1 
        A = pos(j-1,:); % Last joint in pos of last link pos
      end

      % Check if C (obstacle) is on AB
      AB = B - A;
      AC = C - A;

      ABxAC = AB(1)*AC(2) - AB(2)*AC(1);
      
      % if AB AC is aligned
      if abs(ABxAC) < 0.3        % if C is between A and B
        k_AC = dot(AB, AC);
        k_AB = dot(AB, AB);
        if k_AC >= 0 && k_AC <= k_AB

          % Link j is in contact with the obstacle

          % Distance from joint to obstacle:
          l_to_obs = norm(AC);

          % Jacobian
          all_Jc = Jc_func(q_sim', l_to_obs);
          Jc = all_Jc(:,:,j);

          % Calculate torque from contact/collision

          % f maa vaere negativ om hindringen ligger under!!
          %%
          
          %Jc_ps_inv = Jc'/(Jc*Jc');
          
          Jct = Jc'; % Transpose of Jc
          Jct_ps_inv = Jct'*pinv(Jct*Jct'); % Pseudo inverse of transpose
          
          % Coefficient corresponding to which side of the link the
          % obstacle is lying
          c = 1;
          obs_x = c(1);
          obs_y = C(2);
          link_pos = A + (B-A)*l_to_obs/l; % point on link perp. to obstacle
          link_x = link_pos(1);
          link_y = link_pos(2);
          
          if abs(link_x-obs_x) > abs(link_y-obs_y) % Decide based on left/right
              if obs_x > link_x
                  c = -1;
              end
          else % Decide based on over/under
              if obs_y > link_y
                 c = -1; 
              end
          end
          
          % Force acting on obstacle
          f_link = Jct_ps_inv*tau(1:n);
          f_obs = c*norm(f_link)*[-sin(q_sim(j)); cos(q_sim(j))];
                    
          % Torque from obstacle
          tauc = tauc + Jc'*f_obs; % Er dette lov???????
          %%
        end
      end
    end

  end
  
  tauc = [tauc; 0; 0];
end