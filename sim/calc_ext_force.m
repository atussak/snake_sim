function tau_ext = calc_ext_torque(M, C, tau, q, qd, in_contact)
    
    global n l_to_obstacles obstacle_coords
    
    N = n + 2;
    tau_ext = zeros(N,1);
    
    for i = 1:n     % every link
        if in_contact(i)
            all_Jc  = Jc_func(q');
            Jc      = all_Jc(:,:,i);
            all_Jcd = Jcd_func(q');
            Jcd     = all_Jcd(:,:,i);

            EF      = obstacle_coords(i,:);

            aq      = Jc_d*qd;

            r_d     = (Jc*qd);
            EF_d    = r_d';
            arF     = EF_d*r_d;

            b1      = tau - C;
            b2      = -EF*aq - arF;

            K       = inv(EF*Jc*M\Jc'*EF');

            f_ext   = -K*(b2 - EF*Jc*M\b1);
            tau_ext = tau_ext + Jc*f_ext;
        end
    end
end