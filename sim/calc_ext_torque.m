function tau_ext = calc_ext_torque(M, C, tau, q, qd, in_contact)
    
    global n N obstacle_coords Jc_func Jcd_func
    
    tau_ext = zeros(N,1);
    
    for i = 1:n     % every link
        if in_contact(i)
            all_Jc  = Jc_func(q');
            Jc      = all_Jc(:,:,i);
            all_Jcd = Jcd_func(q', qd');
            Jcd     = all_Jcd(:,:,i);

            EF      = obstacle_coords(i,:);

            aq      = Jcd*qd;

            r_d     = (Jc*qd);
            EF_d    = r_d';
            arF     = EF_d*r_d;

            b1      = tau - C;
            b2      = -EF*aq - arF;

            k1 = EF*Jc;
            k2 = pinv(M);
            k3 = Jc'*EF';
            K       = pinv(k1*k2*k3);

            f_ext   = -K*(b2 - EF*Jc*k2*b1);
            f = [0; f_ext];
            tau_ext = tau_ext + Jc'*f;
        end
    end
end