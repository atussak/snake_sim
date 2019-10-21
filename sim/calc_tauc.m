function tauc = calc_tauc(pos, q_sim, tau)

  global n num_obstacles obstacle_coords Jc_func

  tauc = zeros(n, 1);

  % For every obstacle
  for i = 1:num_obstacles
    % Obstacle coordinate
    C = obstacle_coords(i,:);

    % For every link
    for j = 1:n
      % Link line segment A-B
      B = pos(j,:);
      A = [0 0]; % Last joint in origin if first link
      if j ~= 1 
        A = pos(j-1,:); % Last joint in pos of last link pos
      end

      % Check if C is on AB
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

          % f må være negativ om hindringen ligger under!!
          %%
          %f = -1*[1; 2]; %?????????
          
          Jc_ps_inv = Jc'/(Jc*Jc');
          tauc = (eye(n) - Jc'*Jc_ps_inv')*tau
          
          %tauc = tauc + Jc'*f % Er dette lov???????
          %%
        end
      end
    end

  end
  tauc = zeros(n, 1);
end