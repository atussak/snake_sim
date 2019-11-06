function tau = saturate(tau_control)
    global n
    
    max = 0.1;
    
    tau = zeros(n+2,1);
    
    for i = 2:n
       if tau_control(i) > max || tau_control(i) < -max
           tau(i) = sign(tau_control(i))*max;
       else
           tau(i) = tau_control(i);
       end
    end

end