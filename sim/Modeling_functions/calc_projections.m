function [P_af, P_ap, contact, in_contact] = calc_projections(pos, k)

    % --------------------------------------------------------
    % Find out which links and obstacles are in contact.
    % Find out the distance between this contact and the last
    % joint.
    % Calculate the corresponding contact Jacobian and 
    % projection matrices mapping into the allowable position-
    % and force space.
    % --------------------------------------------------------
    
    
  global n N num_obstacles obstacle_coords Jc_func q obstacle_radius
  
  contact = false;
  num_contacts = 0;
  in_contact = zeros(1,n);

  Jc = zeros(2,N,n);
  
  % Allowable force and position space projectors
  P_af = eye(N);
  P_ap = eye(N);
  
  % Stack of matrices
  S_P_af = [];
  S_P_ap = [];
 
  % For every obstacle
  for obs = 1:num_obstacles
    P_af = eye(N);
    P_ap = eye(N);
      
    % Obstacle coordinate
    C = obstacle_coords(obs,:);

    % For every link
    for link = 1:n
      % Link line segment A-B
      B = pos(link,:);
      A = [q(n+1,k) q(n+2,k)]; % Last joint in origin if first link
      if link ~= 1 
        A = pos(link-1,:); % Last joint in pos of last link pos
      end

      % Check if C (obstacle) is on AB
      AB = B - A;
      AC = C - A;

      ABxAC = AB(1)*AC(2) - AB(2)*AC(1);
      
      % if AB AC is aligned
      if abs(ABxAC) < obstacle_radius      % if C is between A and B
        k_AC = dot(AB, AC);
        k_AB = dot(AB, AB);
        if k_AC >= 0 && k_AC <= k_AB

          %% Link j is in contact with the obstacle
          contact = true;
          in_contact(link) = true;
          num_contacts = num_contacts + 1;
          
          % Distance from joint to obstacle:
          l_to_obs = norm(AC);
          q(n+2+link,k) = l_to_obs;
          q(n+2+link,k) = l_to_obs;
          q(n+2+link,k+1) = q(n+2+link,k); % To be used in finding tau_ext
          
          % Jacobian
          all_Jc = Jc_func(q(:,k)');
          Jc(:,:,link) = all_Jc(:,:,link);
          
          P_af = (pinv(Jc(:,:,link))*Jc(:,:,link))';
          P_ap = eye(N) - pinv(Jc(:,:,link))*Jc(:,:,link);
          
          S_P_af = [S_P_af P_af];
          S_P_ap = [S_P_ap P_ap];
        end
      end
    end

  end
  
  num_contacts;
  
  if contact
      % Union of all P_af
      P_af = S_P_af*pinv(S_P_af);

      % Intersect of all P_ap
      P_ap = S_P_ap(:,1:N)*pinv(S_P_ap(:,1:N));
      for i = 2:num_contacts
          P_temp = S_P_ap(:,N*(i-1)+1:N*i);
          P_temp = P_temp*pinv(P_temp);
          P_ap = 2*(P_ap - P_ap*pinv(P_ap + P_temp)*P_ap);
      end
  end

end