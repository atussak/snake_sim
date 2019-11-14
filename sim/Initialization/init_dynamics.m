
% --------------------------------------------------------
% Find symbolic expressions for the M- and C matrices from
% the classical robot dynamic equation.
% --------------------------------------------------------

% Snake robot specifications
global n m l N

% Joint angles and velocities
global q qd qdd Q Qd Qdiff
syms t

% All virtual and real joints
N = n + 2 + n; % n   : one joint per link
               % + 2 : for the virtual x- and y-coordinates
               % + n : length to every possible obstacle from last joint
               %       (maximum one obstacle per link)

Q       = sym('q'  , [1 N]); % time dependency specified
q       = sym('q'  , [1 N]); % time dependency not specified
Qd      = sym('qd' , [1 N]); % time dependency specified
qd      = sym('qd' , [1 N]); % time dependency not specified
qdd     = sym('qdd', [1 N]); % time dependency not specified

% Make symbols time dependent for possibility to differentiate wrt time
for i = 1:N
    syms(sprintf('q%d(t)', i))
    syms(sprintf('qd%d(t)', i))
    Q(i) = symfun(eval(sprintf('q%d(t)', i)), t);
    Qd(i) = symfun(eval(sprintf('qd%d(t)', i)), t);
end

Qdiff  = diff(Q,t);
Qddiff = diff(Qd,t);

%% Cartesian positions
X = sym('x', [1 n]);
Y = sym('y', [1 n]);

% Doesn't include the position of the virtual links as they don't have any
% mass and thus no kinetic energy.
for link = 1:n
   X(link) = Q(n+1);
   Y(link) = Q(n+2);
   for j = 1:link
      q_temp = sym(0);
      for k = 1:j % For absolutte vinkler
         q_temp = q_temp + Q(k); 
      end
      if j == link % To get to the center of mass (middle) of the link
          X(link) = X(link) + (l/2)*cos(q_temp);
          Y(link) = Y(link) + (l/2)*cos(q_temp);
      else
          X(link) = X(link) + l*cos(q_temp);
          Y(link) = Y(link) + l*sin(q_temp);
      end
   end
end

%% Cartesian velocities

Xd = diff(X, t);
Yd = diff(Y, t);

for j = 1:N
    Xd = subs(Xd, Qdiff(j), Qd(j));
    Yd = subs(Yd, Qdiff(j), Qd(j));
end

%% Kinetic energy

K = sym('K', [1 n]);

for link = 1:n
   qd_tot = sym(0);
   for j = 1:link % sum all joint velocities up until current joint
               % to get absolute joint velocity
     qd_tot = qd_tot + Qd(j);
   end
   
   % Sum of translational- and rotational kinetic energy for the link
   % (moment of inertia of rod: 1/12 * ml^2)
   K(link) = 0.5*m*(Xd(link)^2 + Yd(link)^2) + 0.5*l^2/12*m*qd_tot^2;
end

%% Euler Lagrange

L = sum(K(:));

Lq    = sym('Lq'  , [1 N]);
Lqd   = sym('Lqd' , [1 N]);
Lqdt  = sym('Lqdt', [1 N]);

% Replace time dependent symbols with variables 
% for differentiation to work
for j = 1:N
    L = subs(L, Qd(j), qd(j));
    L = subs(L, Q(j), q(j));
end

for i = 1:N
    Lqd(i)   = diff(L, qd(i));
    Lq(i)    = diff(L, q(i));

    % Make variables time dependent again to differentiate wrt time
    for j = 1:N
        Lqd(i) = subs(Lqd(i), qd(j), Qd(j));
        Lqd(i) = subs(Lqd(i), q(j), Q(j));
    end
    Lqd(i) = simplify(Lqd(i));

    Lqdt(i) = diff(Lqd(i), t);
    Lqdt(i) = simplify(Lqdt(i));
end

tau = Lqdt - Lq;

for i = 1:N
    for j = 1:N
        tau(i) = subs(tau(i), Qdiff(j), qd(j));
        tau(i) = subs(tau(i), Qddiff(j), qdd(j));
    end
    for j = 1:N
        tau(i) = subs(tau(i), Q(j), q(j));
        tau(i) = subs(tau(i), Qd(j), qd(j));
    end
end

%% Extract coefficients for finding EoM characteristic eqs

global M C

M = sym('m%d%d', [N N]);
C = sym('c%d', [1 N]);

for i = 1:N
    for j = 1:N
        M(i,j) = diff(tau(i), qdd(j));
    end
end

for i = 1:N
   C(i) = simplify(tau(i) - sum(M(i,:).*qdd));
end


%% Make functions for faster calculation at insertion of numbers
global M_func C_func
M_func = matlabFunction(M, 'vars', {q});
C_func = matlabFunction(C, 'vars', {q, qd, qdd});




