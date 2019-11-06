global n q0 start
start = true;

h       = 0.01;             % sample time (s)
simTime = 50;              % simulation duration in seconds
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
head_pos  = zeros(2,Ns);
tau_motor = zeros(N,1); % motor torque
tau       = zeros(N,1);
P_af = zeros(N,N);
P_ap = zeros(N,N);

% Initial values
q(:,1)    = q0;
q_ref     = q0;
%q_ref(2)  = 0;
%q_ref(3)  = pi/5;
q_ref(4)  = -pi/2;

contact = false;

error    = zeros(N,Ns);
error_d  = zeros(N,Ns);


%% Main simulation loop
for k = 1:Ns-1
  t(k+1) = k*h;
  
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
  
  % Computed torque control
  qdd_ref = zeros(N,1);
  kd = 0.8;
  kp = 0.5;
  tau_control = M*(qdd_ref + kd*error_d(:,k) + kp*error(:,k)) + C';
    
  % Saturate control torque
  tau = saturate(tau_control);

  % Project torque onto the allowable force space
  if contact
      tau = P_af*tau;
      tau = saturate(tau);
  end
  
  % Calculate joint acceleration
  q_dot_dot(:,k) = M\(tau - C');
  
  % Euler integration
  q_dot(:,k+1)     = q_dot(:,k) + q_dot_dot(:,k)*h;
  if contact
    q_dot(:,k+1) = P_ap*q_dot(:,k+1);
    q_dot(:,k+1)
  end
  q(:,k+1)         = q(:,k) + q_dot(:,k)*h;
  
  % Calculate link coordinates
  pos = kinematics(q(:,k));
  head_pos(:,k) = pos(n,:)';

  % Calculate torque from contact
  [P_af, P_ap, contact] = calc_tauc(pos, q(:,k));
  
  x0 = q(n+1,k); 
  y0 = q(n+2,k);
  
  % Control
  error(:,k+1)    = q_ref - q(:,k);
  error(1,k+1)=0; error(n+1:n+2,k+1) = 0;
  error_d(:,k+1)  = (error(:,k+1)-error(:,k))/h; %-q_dot(:,k);
  
  % Visualize robot
  visualize(pos, x0, y0);
end

% plot_robot_data(q, q_dot, q_dot_dot, head_pos, tauc, t);





