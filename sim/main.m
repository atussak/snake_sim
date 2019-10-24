global n q0 start
start = true;

h       = 0.01;             % sample time (s)
simTime = 100;              % simulation duration in seconds
Ns      = simTime/h;        % number of samples

N = n + 2;

% Allocate memory
M = zeros(N,N);
C = zeros(N,1);

% Initial values
q         = q0;
q_dot     = zeros(N,1);
q_dot_dot = zeros(N,1);

tau_free       = zeros(N,1); % motor torque
tau_free(2,1)  = 0.01;
tauc           = zeros(N,1); % torque from constraint
tau            = zeros(N,1);

% tau(1,1) has to be zero at all times as this is only a passive joint

%% Main simulation loop
for k = 1:Ns-1
  
%   q(n+1) = -0.3;
%   q(n+2) = 0.5;  
  
  tau = tau_free + tauc;
  
  % Calculate dynamics matrices
  M = M_func(q');
  C = C_func(q', q_dot');   
  
  % Calculate joint acceleration
  q_dot_dot = pinv(M)*(tau - C');   
  
  % Euler integration
  q_dot     = q_dot + q_dot_dot*h;
  q         = q + q_dot*h;
  
  % Displacement of robot as a result of q moving both prev and prec link
%   q_diff = q_dot*h;
%   q(n+1) = q(n+1) + sum(sin(q_diff./2)); % x
%   q(n+2) = q(n+2) + sum(q(n+1)/tan(pi/2-q_diff/4)); % y
  
  % Links can't cross each other
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
  
  x0 = 0;%q(n+1);
  y0 = 0;%q(n+2);
  
  % Visualize robot
  visualize(pos, x0, y0);
end