function omegadot = angular_acceleration(input,omega, I, L, b, k,mp,xp,yp)
tau = torques(input, L, b, k,mp,xp,yp);
omegadot = I\(tau - cross(omega, I * omega));
end

