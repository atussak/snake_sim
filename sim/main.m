
global n N q0 start q
start = true; % For initializing visual simulator

% Simulation variables
h        = 0.001;            % sample time (s)
simTime  = 20;             % simulation duration in seconds
Ns       = simTime/h;       % number of samples
t        = zeros(1, Ns);    % array of simulation time steps
                            % (updated in loop)

% Allocate memory
M           = zeros(N,N);
C           = zeros(N,1);
q           = zeros(N,Ns);
q_d         = zeros(N,Ns);
q_dd        = zeros(N,Ns);
q_ref       = zeros(N,Ns);
tau         = zeros(N,Ns);
error       = zeros(N,Ns);
error_d     = zeros(N,Ns);
in_contact  = zeros(1,n);
P_af        = zeros(N,N);
P_ap        = zeros(N,N);
pos         = zeros(n,2);
mid_pos     = zeros(n,2);
head_pos    = zeros(2,Ns); % For plotting
energy      = zeros(1,Ns);
momentum    = zeros(2,Ns);

% Initial values
q(:,1)      = q0;

% Assume no contact in the first iteration
contact = false;


%% Main simulation loop
for k = 1:Ns-1
  t(k+1) = k*h;
  
  % Constrain links from overlapping
  [q(:,k), q_d(:,k), q_dd(:,k)] = constrain(q(:,k), q_d(:,k), q_dd(:,k));
  
  % Calculate dynamics matrices
  M = M_func(q(:,k)');
  C = C_func(q(:,k)', q_d(:,k)', q_dd(:,k)');
  
  % Computed torque control
  tau(:,k) = computed_torque_control(M, C', error(:,k), error_d(:,k));

  % Calculate joint acceleration
  q_dd(:,k) = pinv(M)*(tau(:,k) - C');
  
  % Euler integration for joint velocities and angles/positions
  q_d(:,k+1)     = q_d(:,k) + q_dd(:,k)*h;
  if contact
      q_d(:,k+1) = P_ap*q_d(:,k+1);
  end
  q(:,k+1)         = q(:,k) + q_d(:,k)*h;
  
  % Calculate link cartesian coordinates
  last_mid_pos = mid_pos;
  mid_pos = calculate_mid_pos(q(:,k)); 
  pos = calculate_pos(q(:,k));
  head_pos(:,k) = pos(n,:)';

  
  % Calculate velocity and momentum
  vel = (mid_pos - last_mid_pos)./h;
  for i = 1:n
     momentum(:,k) = momentum(:,k) + m*vel(i,:)';
  end
  
  % Calculate projection matrices
  [P_af, P_ap, contact, in_contact] = calculate_projections(pos, k);

  
  % Find the desired joint angles based on the given path
  % Variable for visualization. The indexing referds to the endpoint
  % of the corresponding link. Index n+1 is for the tail projection.
  proj_points = zeros(n+1,2);
  tail_pos = [q(n+1,k) q(n+2,k)];
  proj_points(5,:) = get_point_on_path(tail_pos);
  proj_points(1,:) = get_point_on_path(pos(1,:));
  for i = 2:n
     proj_point = get_point_on_path(pos(i,:));
     q_ref(i,k+1) = get_path_reference_angle(pos(i-1,:), pos(i,:), proj_point, q(i,k),i);
     proj_points(i,:) = proj_point;
  end

    
  % Calculate error for the controller
  error(2:n,k+1)    = q_ref(2:n,k+1) - q(2:n,k+1); % No error in unactuated joints
  error_d(2:n,k+1)  = -q_d(2:n,k+1);
  
  % Calculate energy of robot
  energy(k) = calculate_energy(q_d(:,k), mid_pos, last_mid_pos, h);
  
  % Visualize robot
  x0 = q(n+1,k);
  y0 = q(n+2,k);
  visualize(pos, x0, y0, proj_points, t(k));
end

%plot_robot_data(q, q_d, q_dd, q_ref, head_pos, tau, error, energy, momentum, t);


