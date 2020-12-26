function a = acceleration(inputs, angles, xdot, m, g, k, kd)
gravity = [0; 0; g];
R = [
    cos(angles(3))*sin(angles(2))*cos(angles(1)) + sin(angles(3))*sin(angles(1)); 
    sin(angles(3))*sin(angles(2))*cos(angles(1)) - cos(angles(3))*sin(angles(1));
    cos(angles(2))*cos(angles(1));
    ];
T = thrust(inputs, k);
Fd = -kd*xdot;
a = gravity + 1/m*R*T(3) + 1/m*Fd;
end

