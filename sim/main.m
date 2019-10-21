global n start
start = true;

h       = 0.01;             % sample time (s)
simTime = 100;              % simulation duration in seconds
N       = simTime/h;        % number of samples


% Allocate memory
M = zeros(n,n);
C = zeros(n,1);

% Initial values
q         = zeros(n,1);
q_dot     = zeros(n,1);
q_dot_dot = zeros(n,1);

tau_free       = zeros(n,1);
tau_free(1,1)  = 0.2;
%tau(3,1)      = 0.1;
tauc           = zeros(n,1);
tau            = zeros(n,1);

%% Main simulation loop
for k = 1:N-1
  
  if tauc == 0
    tau = tau_free;
  else
    tau = tauc;
  end
  
  % Calculate dynamics matrices
  M = M_func(q');
  C = C_func(q', q_dot');
  
  % Calculate joint acceleration
  q_dot_dot = M\(tau - C');
  % Euler integration
  q_dot     = q_dot + q_dot_dot*h;
  q         = q + q_dot*h;
  
  for i = 2:n
    if q(i) > pi || q(i) < -pi
      q(i) = pi*sign(q(i))*1;%0.99;
      q_dot(i) = 0;
      q_dot_dot(i) = 0;
    end
  end
  
  % Calculate link coordinates
  pos = kinematics(q);
  
  % Calculate torque from contact
  tauc = calc_tauc(pos, q, tau_free);
  
  % Visualize robot
  visualize(pos);
end