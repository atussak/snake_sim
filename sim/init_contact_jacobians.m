global q qd Q Qdiff n N l Jc_func Jcd_func

% We can have n possible contact Jacobians
% The distance from the joint to the contact point is to be found in the
% last elements of q.

all_Jc  = sym('Jc%d', [2 N n]);
Jc = sym('Jc%d', [2 N]);

all_Jcd  = sym('Jc%d', [2 N n]);
Jcd = sym('Jc%d', [2 N]);

contact_index = 0; % Keep track of 

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
      x = x + q(n+2+i)*cos(q_temp);
      y = y + q(n+2+i)*sin(q_temp);
    else
      x = x + l*cos(q_temp);
      y = y + l*sin(q_temp);
    end
  end

  % Differentiate wrt. the generalized coordinates
  for j = 1:N
    Jc(1,j) = diff(x, q(j));
    Jc(2,j) = diff(y, q(j));
  end
  
  Jc = simplify(Jc);
  all_Jc(:,:,i)  = Jc;
  
  %% Derivative of Jacobian
  % Make variables time dependent to diff wrt time
  for row = 1:2
      for col = 1:N
          for j = 1:N
            Jc(row,col) = subs(Jc(row,col), q(j), Q(j));
          end
      end
  end
  
  for row = 1:2
      for col = 1:N
          Jcd(row,col) = diff(Jc(row,col), t);
      end
  end
  Jcd;
  
  for row = 1:2
      for col = 1:N
          for j = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Qdiff(j), qd(j));
          end
          for j = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Q(j), q(j));
            Jcd(row, col) = subs(Jcd(row, col), Qd(j), qd(j));
          end
      end
  end
  
  all_Jcd(:,:,i) = Jcd;
  
end

Jc_func  = matlabFunction(all_Jc, 'vars', {q});
Jcd_func = matlabFunction(all_Jcd, 'vars', {q, qd});