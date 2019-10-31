function tauc = calc_tauc(pos, q_sim, tau, qd_prev, qd_sim, h)

  global n m l num_obstacles obstacle_coords Jc_func

  N = n + 2;
  
  tauc = zeros(N, 1);

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
      if abs(ABxAC) < 0.1        % if C is between A and B
        k_AC = dot(AB, AC);
        k_AB = dot(AB, AB);
        if k_AC >= 0 && k_AC <= k_AB

          % Link j is in contact with the obstacle
            
          % Distance from joint to obstacle:
          l_to_obs = norm(AC);

          % Jacobian
          all_Jc = Jc_func(q_sim', l_to_obs);
          Jc = all_Jc(:,:,j);
          xd_prev = Jc*qd_prev;
          xd = Jc*qd_sim;
          xdd = (xd - xd_prev)/h;
          f_obs = xdd*m;
          %Jct_ps_inv = pinv(Jc'); % Pseudo inverse of transpose
          
          % Coefficient corresponding to which side of the link the
          % obstacle is lying
          c = 1;
%           obs_x = C(1);
%           obs_y = C(2);
%           link_pos = A + (B-A)*l_to_obs/l; % point on link perp. to obstacle
%           link_x = link_pos(1);
%           link_y = link_pos(2);
%           
%           if abs(link_x-obs_x) < abs(link_y-obs_y) % Decide based on left/right
%               if obs_x < link_x
%                   "Obstacle left"
%                   c = -1;
%               end
%           else % Decide based on over/under
%               if obs_y < link_y
%                  "Obstacle under" 
%                  c = -1; 
%               end
%           end
%           
          % Force acting on obstacle
          %f_link = Jct_ps_inv*tau;
          f_link = c*norm(f_obs)*[-sin(q_sim(j)); cos(q_sim(j))];
          
          % Make sure the force acting back on the link has the opposite
          % sign than the one acting on the obstacle.
%           if f_obs(1)*f_link(1) > 0
%              f_link(1) = -1*f_link(1); 
%           end
%           if f_obs(2)*f_link(2) > 0
%              f_link(2) = -1*f_link(2); 
%           end
          f_obs;
          f_link;
          % Torque from obstacle
          tauc = tauc + Jc'*f_link;
          
        end
      end
    end

  end
  
%   tauc(1) = 0;
%   tauc(n+1) = 0;
%   tauc(n+2) = 0;
  
end