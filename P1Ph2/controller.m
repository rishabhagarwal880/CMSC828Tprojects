function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
phi_des = 0;
theta_des = 0;
psi_des = 0;

roll =qd{1}.euler(1);
pitch =qd{1}.euler(2);
yaw =qd{1}.euler(3);
p_angle = qd{1}.omega(1);
q_angle =qd{1}.omega(2);
r_angle =qd{1}.omega(3);

pos_des = qd{1}.pos_des;
vel_des = qd{1}.vel_des;
acc_des = qd{1}.acc_des;

pos_e = pos_des-qd{1}.pos;
vel_e = vel_des-qd{1}.vel;
acc_e = acc_des;


kp = 20;
kp_ang = 200;
kd = 5;
kd_ang = 25;

acc_final = acc_e+kd*vel_e+kp*pos_e;


m = params.mass;
g = params.grav;
F_des = m*g + m*(acc_final(3));

phi_des = 1/g*(acc_final(1)*sin(yaw)-acc_final(2)*cos(yaw));
theta_des = 1/g*(acc_final(1)*cos(yaw)+acc_final(2)*sin(yaw));

% Thurst
F = F_des;


% Moment
M    = zeros(3,1); % You should fill this in
M = params.I*[kp_ang*(phi_des-roll)+kd_ang*(-p_angle); ...
    kp_ang*(theta_des-pitch)+kd_ang*(-q_angle); kp_ang*(psi_des-yaw)+kd_ang*(-r_angle)];
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
