
% --------------------------------------------------------
% Calculates the transformation matrix
% --------------------------------------------------------

global T_func q n l
all_T = sym('T%d', [4 4 n]);

T      = eye(4);
D      = eye(4);
D(1,4) = l; % link length

% Displacement from virtual translational joints
D_0      = sym(eye(4));
D_0(1,4) = q(n+1);
D_0(2,4) = q(n+2);

T = T*D_0;

for i = 1:n % link number
    R = [cos(q(i)) -sin(q(i)) 0 0;
         sin(q(i))  cos(q(i)) 0 0;
         0          0         1 0;
         0          0         0 1];
    
    T = T*R*D;
    all_T(:,:,i) = simplify(T);
end

T_func = matlabFunction(all_T, 'vars', {q});

% --------------------------------------------------------
% Save general rotation (around z) and translation
% matrices for later use
% --------------------------------------------------------

global Rz_func D_func
syms theta dist_x dist_y

Rz = [cos(theta) -sin(theta) 0 0;
      sin(theta)  cos(theta) 0 0;
      0          0           1 0;
      0          0           0 1];

D = sym(eye(4));
D(1,4) = dist_x;
D(2,4) = dist_y;

Rz_func = matlabFunction(Rz, 'vars', {theta});
D_func  = matlabFunction(D, 'vars', {dist_x, dist_y});

