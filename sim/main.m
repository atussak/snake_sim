addpath('Initialization')
addpath('Control_functions')
addpath('Modeling_functions')

global n N q0 start q
start = true; % For initializing visual simulator

% Simulation variables
h        = 0.01;             % sample time (s)
simTime  = 20;              % simulation duration in seconds
Ns       = simTime/h;        % number of samples
t        = zeros(1, Ns);     % array of simulation time steps
                             % (updated in loop)

% Allocate memory
M           = zeros(N,N);
C           = zeros(N,1);
q           = zeros(N,Ns);
q_d         = zeros(N,Ns);
q_dd        = zeros(N,Ns);
tau         = zeros(N,Ns);
error       = zeros(N,Ns);
error_d     = zeros(N,Ns);
in_contact  = zeros(1,n);
P_af        = zeros(N,N);
P_ap        = zeros(N,N);

head_pos    = zeros(2,Ns); % For plotting

% Initial values
q(:,1)    = q0;

% Reference values
q_ref     = q0;
q_ref(4)  = -pi/2;

% Assume no contact in the first iteration
contact = false;

bend_link = 4;
part2 = false;


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
  pos = kinematics(q(:,k));
  head_pos(:,k) = pos(n,:)';

  % Calculate projection matrices
  [P_af, P_ap, contact, in_contact] = calc_projections(pos, k);
  
%   % Change trajectory
%   if abs(error(bend_link,k)) < 0.001 && k > 1
%       if bend_link > 2 && ~part2
%           q_ref(bend_link-1) = -pi/2;
%           q_ref(bend_link) = 0;
%           bend_link = bend_link - 1;
%       else
%           q_ref(2) = 0;
%           part2 = true;
%           bend_link = 4;
%           q_ref(bend_link) = pi/2;
%       end
% %       if part2 && bend_link > 2
% %           q_ref(bend_link-1) = pi/4
% %           q_ref(bend_link) = 0;
% %           bend_link = bend_link - 1;
% %       end
%   end
  
%   for i = 2:n
%       q_ref(i) = get_path_reference_angle(pos(i,1),t);
%   end
%   q_ref(n+1) = q_ref(n+1) + 0.0000001;
%     q_ref

    proj_points = zeros(4,2);
    for i = 3:n
       proj_point = get_point_on_path(pos(i,:));
       q_ref(i) = get_path_reference_angle(pos(i,:), proj_point, q(i,k));
        proj_points(i,:) = proj_point;
    end
    proj_points;
    q_ref;
  % Calculate error for the controller
  error(:,k+1)    = q_ref - q(:,k);
  error(1,k+1)=0; error(n+1:N,k+1) = 0; % No error in unactuated joints
  error_d(:,k+1)  = (error(:,k+1)-error(:,k))/h;
  error(1:4,k+1)
  % Visualize robot
  x0 = q(n+1,k);
  y0 = q(n+2,k);
  visualize(pos, x0, y0);
end

plot_robot_data(q, q_d, q_dd, head_pos, tau, t);





