function tau = saturate(tau_control)

    % --------------------------------------------------------
    % Make sure the torque doesn't get too high and 
    % set the torques of the unactuated joints to zero
    % --------------------------------------------------------

    global n N max_tau
        
    tau = zeros(N,1);
    
    for i = 2:n
       if tau_control(i) > max_tau || tau_control(i) < -max_tau
           tau(i) = sign(tau_control(i))*max_tau;
       else
           tau(i) = tau_control(i);
       end
    end

end