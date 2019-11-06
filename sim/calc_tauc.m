function [tauc, contact, Jc] = calc_tauc(pos, q_sim, qdd_sim, tau, M_dyn, C_dyn)

  global n m l num_obstacles obstacle_coords Jc_func

  N = n + 2;
  
  contact = false;
  tauc = zeros(N,1);
  Jc = zeros(2,N+1,n);

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

          %% Link j is in contact with the obstacle
          contact = true;  
          
          % Distance from joint to obstacle:
          l_to_obs = norm(AC);

          % Jacobian
          all_Jc = Jc_func(q_sim', l_to_obs);
          Jc(:,:,j) = all_Jc(:,:,j);
          
          tau_obs = calc_external_f(M_dyn, C_dyn, tau, Jc, qdd_sim);
          
          
          % Coefficient corresponding to which side of the link the
          % obstacle is lying
          % right side: 1
          % left side : -1
          c = 1;

          % Force acting on obstacle
          theta = 0;
          for p = 1:j
              theta = theta + q_sim(p);
          end
          
          %f_link = c*norm(f_obs)*[-sin(theta); cos(theta)];
          
          % Make sure the force acting back on the link has the opposite
          % sign than the one acting on the obstacle.
%           if f_obs(1)*f_link(1) > 0
%              f_link(1) = -1*f_link(1); 
%           end
%           if f_obs(2)*f_link(2) > 0
%              f_link(2) = -1*f_link(2); 
%           end
%           f_obs;
%           f_link;
          % Torque from obstacle
          tauc = tauc + tau_obs; %Jc'*f_link;
          
        end
      end
    end

  end
  
%   tauc(1) = 0;
%   tauc(n+1) = 0;
%   tauc(n+2) = 0;
  
end