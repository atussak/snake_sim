global n q0 start
start = true;

h       = 0.01;             % sample time (s)
simTime = 60;              % simulation duration in seconds
Ns      = simTime/h;        % number of samples
t       = zeros(1, Ns);     % array of simulation time steps
                            % (updated in loop)

N = n + 2; % number of all virtual and real links

% Allocate memory
M = zeros(N,N);
C = zeros(N,1);
q         = zeros(N,Ns);
q_dot     = zeros(N,Ns);
q_dot_dot = zeros(N,Ns);
head_pos = zeros(2,Ns);
tau_motor       = zeros(N,1); % motor torque
tauc            = zeros(N,Ns); % torque from constraint
tau             = zeros(N,1);

% Initial values
q(:,1)    = q0;
tau_motor(2,1)  = 0.0;
tau_motor(4,1)  = -0.005;


%% Main simulation loop
for k = 1:Ns-1
  t(k+1) = k*h;
  
  tau = tau_motor + tauc(:,k);
  
  for i = 2:n
    % Links can't cross each other
    % Note: first link and virtual links aren't limited
    if q(i,k) > pi || q(i,k) < -pi
      q(i,k) = pi*sign(q(i,k));
      q_dot(i,k) = 0;
      q_dot_dot(i,k) = 0;
    end
    
  end 
  
  % Calculate dynamics matrices
  M = M_func(q(:,k)');
  C = C_func(q(:,k)', q_dot(:,k)', q_dot_dot(:,k)');   
  
  % Calculate joint acceleration
  q_dot_dot(:,k) = M\(tau - C');
  
  % Euler integration
  q_dot(:,k+1)     = q_dot(:,k) + q_dot_dot(:,k)*h;
  q(:,k+1)         = q(:,k) + q_dot(:,k)*h;
  
  % Calculate link coordinates
  pos = kinematics(q(:,k));
  head_pos(:,k) = pos(n,:)';

  % Calculate torque from contact
  tauc(:,k+1) = calc_tauc(pos, q(:,k), tau, q_dot(:,k), q_dot(:,k+1), h);
  %tauc = zeros(N,1);
  
  x0 = q(n+1,k);
  y0 = q(n+2,k);
  
  % Visualize robot
  visualize(pos, x0, y0);
end

plot_robot_data(q, q_dot, q_dot_dot, head_pos, tauc, t);





