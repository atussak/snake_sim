%% Transformation matrix

global T_func q n
all_T = sym('T%d', [4 4 n]);

T = eye(4);
D = eye(4);
D(1,4) = l;

for i = 1:n
    R = [cos(q(i)) -sin(q(i)) 0 0;
         sin(q(i))  cos(q(i)) 0 0;
         0          0         1 0;
         0          0         0 1];
    
    T = T*R*D;
    all_T(:,:,i) = simplify(T);
end

T_func = matlabFunction(all_T, 'vars', {q});