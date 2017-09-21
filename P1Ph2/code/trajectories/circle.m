function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
yaw = 0;
yawdot = 0;

T = 15;
f = .0001;
curve = 2*pi/((pi^.5*erf(f^.5*(T-T/2))/(2*f^.5)-T*exp(-f*T^2/4))-pi^.5*...
    erf(-f^.5*T/2)/(2*f^.5));
c =-curve*pi^.5*erf(f^.5*T/2)/(2*f^.5)+curve*T*exp(-f*T^2/4)+2*pi;
theta = curve*(pi^.5*erf(f^.5*(t-T/2))/(2*f^.5)-t*exp(-f*T^2/4))+c;
theta_dot = curve*(exp(-(f^.5*t-f^.5*T/2)^2)-exp(-f*T^2/4));
theta_ddot = f*curve*(T-2*t)*exp(-.25*f*(T-2*t)^2);

pos = [5*cos(theta); 5*sin(theta); 2.5/(2*pi)*theta];
vel = [-5*(theta_dot)*sin(theta); 5*(theta_dot)*cos(theta); 2.5/(2*pi)*theta_dot];
acc = [-5*(theta_dot)^2*cos(theta); -5*(theta_dot)^2*sin(theta); 2.5/(2*pi)*theta_ddot];


if (t>=T) 

    pos = [5; 0; 2.5];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end


% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
