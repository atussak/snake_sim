
% Snake robot specifications
global n
m = 1; % link mass
l = 1; % link length

% Joint angles and velocities

global q qd
syms t

Q       = sym('q'  , [1 n]); % time dependency specified
q       = sym('q'  , [1 n]); % time dependency not specified
Qd      = sym('qd' , [1 n]); % time dependency specified
qd      = sym('qd' , [1 n]); % time dependency not specified
qdd     = sym('qdd', [1 n]); % time dependency not specified

% make time dependent
for i = 1:n
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

for i = 1:n
   X(i) = 0;
   Y(i) = 0;
   for j = 1:i
      q_temp = sym(0);
      for k = 1:j
         q_temp = q_temp + Q(k); 
      end
      X(i) = X(i) + l*cos(q_temp);
      Y(i) = Y(i) + l*sin(q_temp);
   end
end

% Cartesian velocities

Xd = diff(X, t);
Yd = diff(Y, t);

for j = 1:n
    Xd = subs(Xd, Qdiff(j), Qd(j));
    Yd = subs(Yd, Qdiff(j), Qd(j));
end

% Kinetic energy

K = sym('K', [1 n]);

for i = 1:n
   K(i) = 0.5*m*(Xd(i)^2 + Yd(i)^2); 
end

% Euler Lagrange

L = sum(K(:));

Lq    = sym('Lq'  , [1 n]);
Lqd   = sym('Lqd' , [1 n]);
Lqdt  = sym('Lqdt', [1 n]);

% Replace time dependent symbols with variables 
% for differentiation to work
for j = 1:n
    L = subs(L, Qd(j), qd(j));
    L = subs(L, Q(j), q(j));
end

for i = 1:n
    Lqd(i)   = diff(L, qd(i));
    Lq(i)    = diff(L, q(i));

    % Make variables time dependent again to differentiate wrt time
    for j = 1:n
        Lqd(i) = subs(Lqd(i), qd(j), Qd(j));
        Lqd(i) = subs(Lqd(i), q(j), Q(j));
    end
    Lqd(i) = simplify(Lqd(i));

    Lqdt(i) = diff(Lqd(i), t);
    Lqdt(i) = simplify(Lqdt(i));
end

tau = Lqdt - Lq;

for i = 1:n
    for j = 1:n
        tau(i) = subs(tau(i), Qdiff(j), qd(j));
        tau(i) = subs(tau(i), Qddiff(j), qdd(j));
    end
    for j = 1:n
        tau(i) = subs(tau(i), Q(j), q(j));
        tau(i) = subs(tau(i), Qd(j), qd(j));
    end
end

% Extract coefficients for finding EoM characteristic eqs

global M C

M = sym('m%d%d', [n n]);
C = sym('c%d', [1 n]);

for i = 1:n
    for j = 1:n
        M(i,j) = diff(tau(i), qdd(j));
    end
end

for i = 1:n
   C(i) = simplify(tau(i) - sum(M(i,:).*qdd));
end

global M_func C_func
M_func = matlabFunction(M, 'vars', {q});
C_func = matlabFunction(C, 'vars', {q, qd});




