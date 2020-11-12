# Drone
## This repository contains all the required MATLAB functions and script to run a simulation of a Drone with Payload attached not necessarily at CoG.
## To execute
- Dowload and unzip all the MATLAB codes(*.m files*) into the same folder.
- Then run the MATLAB script ***test.m*** to generate the plots  
  - *Roll, Pitch and Yaw v/s time*
  - *X, Y and Z v/s time*
  - *Thrust from Motor 1,2,3,4 v/s time*
- The values of Mass of Drone, Mass of Payload, Inertial values of Drone, rotor force constant and rotor moment constant can be changed within the MATLAB script ***test.m***
## To change PID values
- The PID control values for Attitude can be changed in ***pid_attitude.m***
- The PID control values for Height can be changed in ***pid_height.m***
- The PID control values for Position can be changed in ***pid_position.m***
