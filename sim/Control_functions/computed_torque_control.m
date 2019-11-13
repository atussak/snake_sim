function tau_control = computed_torque_control(M, C, error, error_d)

    % --------------------------------------------------------
    % Calculate the control torque based on the error in joint
    % values and the time derivative of the error.
    % --------------------------------------------------------

    global N
    
    qdd_ref = zeros(N,1);
    
    % Controller gains
    kd = 0.8;
    kp = 0.5;
    
    tau_control = M*(qdd_ref + kd*error_d + kp*error) + C;
    
    % Saturate control torque
    % Both for restricting torque to actuated joints
    % and for limiting the magnitude.
    tau_control = saturate(tau_control);
    
end