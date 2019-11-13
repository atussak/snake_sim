
% --------------------------------------------------------
% * Find symbolic expressions for the possible 
%   contact Jacobians and their derivatives.
% * We can have n possible contact Jacobians (one per link).
% * The distance from the joint to the contact point is to be
%   found in the last elements of q.
% --------------------------------------------------------


global q qd Q Qdiff n N l Jc_func Jcd_func


% Contact Jacobians
all_Jc  = sym('Jc%d', [2 N n]);
Jc = sym('Jc%d', [2 N]);

% Derivative of contact Jacobians
all_Jcd  = sym('Jc%d', [2 N n]);
Jcd = sym('Jc%d', [2 N]);


for link = 1:n % For every link an obstacle can be in contact with
  
  % Calculate distances from base
  
  % Start at tail position
  x = q(n+1);
  y = q(n+2);
  
  % Add x- and y distance from the links from tail to contact point
  for i = 1:link
    q_temp = sym(0);
    for k = 1:i
       q_temp = q_temp + q(k); 
    end
    if i == link % The link in contact is reached
      x = x + q(n+2+link)*cos(q_temp);
      y = y + q(n+2+link)*sin(q_temp);
    else
      x = x + l*cos(q_temp);
      y = y + l*sin(q_temp);
    end
  end

  % Differentiate wrt. the generalized coordinates
  for i = 1:N
    Jc(1,i) = diff(x, q(i));
    Jc(2,i) = diff(y, q(i));
  end
  
  Jc = simplify(Jc);
  all_Jc(:,:,link)  = Jc;
  
  %% Derivative of Jacobian
  % Make variables time dependent to diff wrt time
  for row = 1:2
      for col = 1:N
          for i = 1:N
            Jc(row,col) = subs(Jc(row,col), q(i), Q(i));
          end
      end
  end
  
  for row = 1:2
      for col = 1:N
          Jcd(row,col) = diff(Jc(row,col), t);
      end
  end
  
  for row = 1:2
      for col = 1:N
          for i = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Qdiff(i), qd(i));
          end
          for i = 1:N
            Jcd(row, col) = subs(Jcd(row, col), Q(i), q(i));
            Jcd(row, col) = subs(Jcd(row, col), Qd(i), qd(i));
          end
      end
  end
  
  all_Jcd(:,:,link) = Jcd;
  
end

%% Make functions for faster calculation at insertion of numbers
Jc_func  = matlabFunction(all_Jc, 'vars', {q});
Jcd_func = matlabFunction(all_Jcd, 'vars', {q, qd});

