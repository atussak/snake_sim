global q n lc l Jc_func %JcT_func

% We can have n possible contact Jacobians
% The distance from the joint to the contact point is generalized as lc

syms lc

N = n + 2;

all_Jc  = sym('Jc%d', [2 N+1 n]);
Jc = sym('Jc%d', [2 N+1]);

% all_Jc  = sym('Jc%d', [1 N n]);
% Jc = sym('Jc%d', [2 N]);

for i = 1:n % For every link an obstacle can be in contact with
  
  % Calculate distances from base
  x = q(n+1);
  y = q(n+2);
  
  for j = 1:i
    q_temp = sym(0);
    for k = 1:j
       q_temp = q_temp + q(k); 
    end
    if j == i
      x = x + lc*cos(q_temp);
      y = y + lc*sin(q_temp);
    else
      x = x + l*cos(q_temp);
      y = y + l*sin(q_temp);
    end
  end
  
  % Rotate to express in obstacle frame
  q_sum = sum(q(1:i));
  R = [cos(q_sum) -sin(q_sum);
       sin(q_sum)  cos(q_sum)];
  pos_b = [x; y];
%   pos_c = R*pos_b;
%   x = pos_c(1);
  
  % Differentiate wrt. the generalized coordinates
  for j = 1:N
    Jc(1,j) = diff(x, q(j));
    Jc(2,j) = diff(y, q(j));
  end
  
  Jc(1,N+1) = diff(x, lc);
  Jc(2,N+1) = diff(y, lc);
  
  Jc = simplify(Jc);
%   Jc = R*Jc;
  
  all_Jc(:,:,i)  = Jc;
end

Jc_func = matlabFunction(all_Jc, 'vars', {q, lc});