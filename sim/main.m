global n q0 start
start = true;

h       = 0.01;             % sample time (s)
simTime = 30;              % simulation duration in seconds
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
tauc      = zeros(N,Ns); % torque from constraint
tau       = zeros(N,1);

% Initial values
q(:,1)    = q0;
q_ref     = q0;
q_ref(2)  = 0;
q_ref(3)  = pi/5;
q_ref(4)  = 0;

contact = false;

error    = zeros(N,Ns);
error_d  = zeros(N,Ns);

Jc = zeros(2,N,n);


%% Main simulation loop
for k = 1:Ns-1
  t(k+1) = k*h;
  %tau_motor(4,1) = tau_motor(4,1) + 0.0001;
%   tau = tau_motor;
%   if contact
%       tau = tauc(:,k);
%   end
  
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
%   qdd_ref = [0; 0; -0.001; -0.002; 0.001; 0];
%   qd_ref  = qdd_ref*h;
%   q_ref   = qd_ref*h;
  qdd_ref = zeros(N,1);
  kd = 0.8;
  kp = 0.5;
  tau_control = M*(qdd_ref + kd*error_d(:,k) + kp*error(:,k)) + C';
  
%   tau_control = [0; 0; -0.01; 0.01; 0.00; 0];
  
  % Saturate control torque
  tau = saturate(tau_control);

%   
%   Jc = Jc(:,:,4);
%   f = pinv(Jc')*(M*qdd_ref + C' - tau);
%   tau2 = -Jc'*f;
%   f_robot = pinv(Jc')*tau;
%   f = f + f_robot;
%   tau = tau + tau2
   
  % Calculate joint acceleration
  q_dot_dot(:,k) = M\(tau - C');
  
  % Euler integration
  q_dot(:,k+1)     = q_dot(:,k) + q_dot_dot(:,k)*h;
  q(:,k+1)         = q(:,k) + q_dot(:,k)*h;
  
  % Calculate link coordinates
  pos = kinematics(q(:,k));
  head_pos(:,k) = pos(n,:)';

  % Calculate torque from contact
  %[tauc(:,k+1), contact, Jc] = calc_tauc(pos, q(:,k), q_dot_dot(:,k), tau, M, C);
  %tauc = zeros(N,1);
  
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





