
% Snake robot specifications
global n m l

% Joint angles and velocities

global q qd qdd
syms t

N = n + 2; % 2 for the virtual x- and y-coordinates

Q       = sym('q'  , [1 N]); % time dependency specified
q       = sym('q'  , [1 N]); % time dependency not specified
Qd      = sym('qd' , [1 N]); % time dependency specified
qd      = sym('qd' , [1 N]); % time dependency not specified
qdd     = sym('qdd', [1 N]); % time dependency not specified

% make time dependent
for i = 1:N
    syms(sprintf('q%d(t)', i))
    syms(sprintf('qd%d(t)', i))
    Q(i) = symfun(eval(sprintf('q%d(t)', i)), t);
    Qd(i) = symfun(eval(sprintf('qd%d(t)', i)), t);
end

Qdiff  = diff(Q,t);
Qddiff = diff(Qd,t);

% Cartesian positions

X = sym('x', [1 n]);
Y = sym('y', [1 n]);

% Doesn't include the position of the virtual links as they don't have any
% mass and thus no kinetic energy.
for i = 1:n
   X(i) = Q(n+1);
   Y(i) = Q(n+2);
   for j = 1:i
      q_temp = sym(0);
      for k = 1:j % For absolutte vinkler
         q_temp = q_temp + Q(k); 
      end
%       X(i) = X(i) + l*cos(q_temp);
%       Y(i) = Y(i) + l*sin(q_temp);
      if j == i % To get to the center of mass (middle) of the link
          X(i) = X(i) + (l/2)*cos(q_temp);
          Y(i) = Y(i) + (l/2)*cos(q_temp);
      else
          X(i) = X(i) + l*cos(q_temp);
          Y(i) = Y(i) + l*sin(q_temp);
      end
   end
end

% Cartesian velocities

Xd = diff(X, t);
Yd = diff(Y, t);

for j = 1:N
    Xd = subs(Xd, Qdiff(j), Qd(j));
    Yd = subs(Yd, Qdiff(j), Qd(j));
end

% Kinetic energy

K = sym('K', [1 n]);

% Cartesian kinetic energy
for i = 1:n
   K(i) = 0.5*m*(Xd(i)^2 + Yd(i)^2) + 0.5*l/2*m*Qd(i)^2;
end

% Euler Lagrange

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

% Extract coefficients for finding EoM characteristic eqs

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

global M_func C_func
M_func = matlabFunction(M, 'vars', {q});
C_func = matlabFunction(C, 'vars', {q, qd, qdd});




