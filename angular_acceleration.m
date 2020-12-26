function omegadot = angular_acceleration(input,omega, I, L, b, k,mp,xp,yp,g)
tau = torques(input, L, b, k,mp,xp,yp,g);
omegadot = I\(tau - cross(omega, I * omega));
end

