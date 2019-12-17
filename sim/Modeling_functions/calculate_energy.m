function energy = calculate_energy(qd, pos, last_pos, h)

    global n m l
    
    energy = 0;
    
    for i = 1:n
        vel = (pos(i,:)-last_pos(i,:))./h;
        I = m*l^2/12;
        energy = energy + 0.5*m*vel*vel' + 0.5*I*sum(qd(1:i))^2;
    end


end