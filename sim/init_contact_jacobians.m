global q n lc l Jc_func %JcT_func

% We can have n possible contact Jacobians
% The distance from the joint to the contact point is generalized as lc

syms lc

all_Jc  = sym('Jc%d', [2 n n]);
Jc = sym('Jc%d', [2 n]);


for i = 1:n
  
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
  
  % Differentiate wrt. the generalized coordinates (q)
  for j = 1:n
    Jc(1,j) = diff(x, q(j));
    Jc(2,j) = diff(y, q(j));
  end
  
  all_Jc(:,:,i)  = simplify(Jc);
  %all_JcT(:,:,i) = (simplify(Jc))';
end

Jc_func = matlabFunction(all_Jc, 'vars', {q, lc});