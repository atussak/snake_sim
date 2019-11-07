function [P_af, P_ap, contact] = calc_projections(pos, q_sim)

  global n num_obstacles obstacle_coords Jc_func

  N = n + 2 + 1;
  
  contact = false;
  num_contacts = 0;

  Jc = zeros(2,N,n);
%   Jc = zeros(1,N,n);
  
  % Allowable force and position space projectors
  P_af = eye(N);
  P_ap = eye(N);
  
  % Stack of matrices
  S_P_af = [];
  S_P_ap = [];
 
  % For every obstacle
  for i = 1:num_obstacles
    P_af = eye(N);
    P_ap = eye(N);
      
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
      if abs(ABxAC) < 0.1      % if C is between A and B
        k_AC = dot(AB, AC);
        k_AB = dot(AB, AB);
        if k_AC >= 0 && k_AC <= k_AB

          %% Link j is in contact with the obstacle
          contact = true;
          num_contacts = num_contacts + 1;
          
          % Distance from joint to obstacle:
          l_to_obs = norm(AC);

          % Jacobian
          all_Jc = Jc_func(q_sim', l_to_obs);
          Jc(:,:,j) = all_Jc(:,:,j);
          
          P_af = (pinv(Jc(:,:,j))*Jc(:,:,j))';
          P_ap = eye(N) - pinv(Jc(:,:,j))*Jc(:,:,j);
          
          S_P_af = [S_P_af P_af];
          S_P_ap = [S_P_ap P_ap];
        end
      end
    end

  end
  
  num_contacts;
  
  if contact
      % Union of all P_af
      P_af = S_P_af*pinv(S_P_af)

      % Intersect of all P_ap
      P_ap = S_P_ap(:,1:N)*pinv(S_P_ap(:,1:N));
      for i = 2:num_contacts
          P_temp = S_P_ap(:,N*(i-1)+1:N*i);
          P_temp = P_temp*pinv(P_temp);
          P_ap = 2*(P_ap - P_ap*pinv(P_ap + P_temp)*P_ap);
      end
  end
  P_af = P_af(1:N-1, 1:N-1);
  P_ap = P_ap(1:N-1, 1:N-1);
end