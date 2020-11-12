% Compute system inputs and updated state.
 % Note that input = [?1, . . ., ?4]
 function [e] = pid_attitude(theta,thetadot,thetaint,thetadesired)
 % Controller gains, tuned by hand and intuition.
 Kd = 12.5;
 Kp = 50;
 Ki = 5;
% Compute errors
 e = Ki*thetaint + Kd * thetadot + Kp*(theta-thetadesired);
 end

