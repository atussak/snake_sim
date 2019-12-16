function [q, q_d, q_dd] = constrain(q_sim, q_d_sim, q_dd_sim)

    % --------------------------------------------------------
    % Constrain links from overlapping
    % Links can't cross each other
    % Note: first link and virtual links aren't limited
    % --------------------------------------------------------

    global n
    
    q       = q_sim;
    q_d     = q_d_sim;
    q_dd    = q_dd_sim;
    
    for i = 2:n
        % Links can not cross each other
        % Note: first link and virtual links aren't limited
        if q_sim(i) > pi || q_sim(i) < -pi
            q(i) = pi*sign(q_sim(i));
            q_d(i) = 0;
            q_dd(i) = 0;
        end
    end
    
end