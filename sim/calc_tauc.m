function [P_af, P_ap, contact] = calc_tauc(pos, q_sim)

  global n num_obstacles obstacle_coords Jc_func

  N = n + 2;
  
  contact = false;

  Jc = zeros(2,N,n);
  
  P_af = zeros(N,N);
  P_ap = zeros(N,N);

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
      if abs(ABxAC) < 0.2        % if C is between A and B
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
          
          P_af = (pinv(Jc(:,:,j))*Jc(:,:,j))';
          P_ap = eye(N) - pinv(Jc(:,:,j))*Jc(:,:,j);
        end
      end
    end

  end
  
end