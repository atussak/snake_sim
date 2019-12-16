addpath('Initialization')
addpath('Control_functions')
addpath('Modeling_functions')
addpath('Visualization_functions')

global n N q0 start q
start = true; % For initializing visual simulator

% Simulation variables
h        = 0.001;            % sample time (s)
simTime  = 15;             % simulation duration in seconds
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

head_pos    = zeros(2,Ns); % For plotting

% Initial values
q(:,1)      = q0;

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
  [P_af, P_ap, contact, in_contact] = calc_projections2(pos, k);

  
  proj_points = zeros(5,2);
  q_ref_temp = 0;
  
  tail_pos = [q(n+1,k) q(n+2,k)];
  proj_points(5,:) = get_point_on_path(tail_pos);
%   q_ref(n+1,k+1) = proj_point(1);
%   q_ref(n+2,k+1) = proj_point(2);
%   tail_ang_ref = get_path_reference_angle2(pos(i,:), tail_pos, proj_point, 0, 1);
  
    proj_points(1,:) = get_point_on_path(pos(1,:));
  for i = 2:n
     proj_point = get_point_on_path(pos(i,:));
     q_ref(i,k+1) = get_path_reference_angle2(pos(i-1,:), pos(i,:), proj_point, q(i,k),i);
     proj_points(i,:) = proj_point;
  end

    
  % Calculate error for the controller
  error(2:n,k+1)    = q_ref(2:n,k+1) - q(2:n,k+1); % No error in unactuated joints
  error_d(2:n,k+1)  = -q_d(2:n,k+1);
  
  % Visualize robot
  x0 = q(n+1,k);
  y0 = q(n+2,k);
  visualize(pos, x0, y0, proj_points, t(k));
end

%plot_robot_data(q, q_d, q_dd, q_ref, head_pos, tau, error, t);

% fontsize_label = 10;
% fontsize_legend = 9;
% textwidth = 12.12364; % cm
% textheight = 16.3757; %cm
% 
% fig = figure('PaperPositionMode', 'Auto', 'Units', 'centimeters', 'Position', [0 0 textwidth 0.50 * textheight]);
% clf
% 
% subplot(2,1,1);
% hold on
% plot(t, q(1,:), 'LineWidth',1);
% plot(t, q(2,:), 'LineWidth',1);
% plot(t, q(3,:), 'LineWidth',1);
% plot(t, q(4,:), 'LineWidth',1);
% legend('$q_1$', '$q_2$', '$q_3$', '$q_4$','Interpreter','latex','FontSize', fontsize_legend)
% title('Joint angles','Interpreter','latex')
% xlabel('$[s]$','Interpreter','latex')
% ylabel('$[rad]$','Interpreter','latex')
% hold off
% 
% subplot(2,1,2); 
% hold on
% plot(t, q_d(1,:), 'LineWidth',1);
% plot(t, q_d(2,:), 'LineWidth',1);
% plot(t, q_d(3,:), 'LineWidth',1);
% plot(t, q_d(4,:), 'LineWidth',1);
% legend('$\dot{q_1}$', '$\dot{q_2}$', '$\dot{q_3}$', '$\dot{q_4}$','Interpreter','latex','FontSize', fontsize_legend)
% title('Joint velocities', 'Interpreter','latex')
% xlabel('$[s]$','Interpreter','latex')
% ylabel('$[rad/s]$','Interpreter','latex')
% hold off
% 
% set(gca,...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',fontsize_label)
% 
% %fig = zoomPlot(fig, [40, 40, 10, 20], [60, 40, 20, 20], 0);
% 
% print_figure(gcf, 'case21')


% fig = figure('PaperPositionMode', 'Auto', 'Units', 'centimeters', 'Position', [0 0 textwidth 0.50 * textheight]);
% clf
% 
% linewidth = 1;
% 
% hold on
% 
% blue = [0.27 0.51 0.71];
% green = [0.24 0.7 0.44];
% orange = [1 0.65 0];
% purple = [0.5 0 0.5];
% 
% % plot(t, q(1,:), 'color', 'b', 'LineWidth',1);
% plot(t, q(2,:), 'color', blue, 'LineWidth',1);
% plot(t, q(3,:), 'color', green, 'LineWidth',1);
% plot(t, q(4,:), 'color', orange, 'LineWidth',1);
% % plot(t, q(5,:), 'color', purple, 'LineWidth',1);
% 
% % plot(t, q_ref(1,:), '--', 'color', 'b', 'LineWidth',1);
% plot(t, q_ref(2,:), '--', 'color', blue, 'LineWidth',1);
% plot(t, q_ref(3,:), '--', 'color', green, 'LineWidth',1);
% plot(t, q_ref(4,:), '--', 'color', orange, 'LineWidth',1);
% % plot(t, q_ref(5,:), '--', 'color', purple, 'LineWidth',1);
% 
% legend('$q_2$', '$q_3$', '$q_4$', '$q_{d,2}$', '$q_{d,3}$', '$q_{d,4}$','Interpreter','latex','FontSize', fontsize_legend)
% title('Joint angles', 'Interpreter','latex')
% xlabel('$s$','Interpreter','latex', 'FontSize', fontsize_label)
% ylabel('$rad$','Interpreter','latex', 'FontSize', fontsize_label)
% hold off
% 
% set(gca,...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',fontsize_label)
% 
% print_figure(gcf, 'case12a')


