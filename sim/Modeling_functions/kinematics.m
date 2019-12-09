function pos = kinematics(q_sim)

    % --------------------------------------------------------
    % Uses the symbolically found transformation matrices to
    % get the cartesian position of the end of every link.
    % --------------------------------------------------------

  % Homogeneous matrices
  global T_func n
    
  T = T_func(q_sim');

  pos = zeros(n,2);
  
  for i = 1:n
    pos(i,:) = T(1:2,4,i);
  end

end