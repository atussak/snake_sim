function tau = saturate(tau_control)
    global n N
    
    max = 0.08;
    
    tau = zeros(N,1);
    
    for i = 2:n
       if tau_control(i) > max || tau_control(i) < -max
           tau(i) = sign(tau_control(i))*max;
       else
           tau(i) = tau_control(i);
       end
    end

end