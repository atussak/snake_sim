function tau_control = computed_torque_control(M, C, q_e, qd_e)

    % --------------------------------------------------------
    % Calculate the control torque based on the error in joint
    % values and the time derivative of the error.
    % --------------------------------------------------------

    global N kp kd
    
    qdd_ref = zeros(N,1);
    
    tau_control = M*(qdd_ref + kd*qd_e + kp*q_e) + C;    
    
    % Saturate control torque
    % Both for restricting torque to actuated joints
    % and for limiting the magnitude.
    tau_control = saturate(tau_control);
    
end