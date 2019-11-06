function tau_ext = calc_external_f(M, C, tau, Jc, qdd)

    global n
    
    N = 2 + n;
%     
%     f_ext = pinv(Jc')*(M*qdd + C' - tau);
%     
%     
    %a = tau';
    tau_ext = zeros(N,1);
%     P_af = (pinv(Jc)*Jc)';
%     tau_ext(2:n) = P_af*tau(2:n);
%     b = tau_ext';

%     A = eye(n+2);
%     A(1) = 0; A(n+1) = 0; A(n+2) = 0;
%     
%     P_f = intersect(A, P_af);
%     
%     f = pinv(Jc')*tau; % force applied to obstacle
%     
%     f = [0.01; 0];
%     tau = Jc'*f;
%     %tau_ext = P_af*(-f) % force on robot projected onto the null space
%                          % of the obstacle
%     tau_ext = -tau;%-P_af*tau
%     tau_ext(1) = 0;
%     tau_ext(n+1) = 0;
%     tau_ext(n+2) = 0;
end