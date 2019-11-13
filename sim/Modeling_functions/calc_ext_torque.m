function tau_ext = calc_ext_torque(M, C, tau, q, qd, qd_prev, in_contact)
    
    global n N obstacle_coords Jc_func Jcd_func C_func
    
    tau_ext = zeros(N,1);
    c = [1 -1 1 1];
    
    
    for i = 1:n     % every link
        if in_contact(i)
            all_Jc  = Jc_func(q');
            Jc      = all_Jc(:,:,i);
%             all_Jcd = Jcd_func(q', qd');
%             Jcd     = all_Jcd(:,:,i);
% 
%             EF      = obstacle_coords(i,:);
% 
%             aq      = Jcd*qd;
% 
%             r_d     = (Jc*qd);
%             EF_d    = r_d';
%             arF     = EF_d*r_d;
% 
%             b1      = tau - C;
%             b2      = -EF*aq - arF;
% 
%             k1 = EF*Jc;
%             k2 = pinv(M);
%             k3 = Jc'*EF';
%             K       = pinv(k1*k2*k3);
% 
%             f_ext   = -K*(b2 - EF*Jc*k2*b1);
%             f = [0; f_ext];
%             tau_ext = tau_ext + Jc'*f;
%             
            h = 0.01; m = 1;
            xd_prev = Jc*qd_prev;
            xd = Jc*qd;
            xdd = (xd-xd_prev)/h;
            f = m*xdd;
            phi = sum(q(1:i));
            fc = c(i)*norm(f)*[-sin(phi); cos(phi)];
            tau_ext = tau_ext + Jc'*fc
            
        end
    end
end