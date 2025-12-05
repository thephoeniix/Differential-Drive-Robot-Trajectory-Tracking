function [traj, deriv] = buildTrajectory(Ts)

    eta = 1;  
    alpha = 4;

    t = 0:Ts:(2*pi*alpha*2);

    x  = eta * sin(t/alpha);
    y  = eta * sin(t/(2*alpha));

    dx  = eta*cos(t/alpha)     * (1/alpha);
    dy  = eta*cos(t/(2*alpha)) * (1/(2*alpha));

    ddx = -eta*sin(t/alpha)     * (1/alpha)^2;
    ddy = -eta*sin(t/(2*alpha)) * (1/(2*alpha))^2;

    traj.t = t; traj.x = x; traj.y = y;
    deriv.dx = dx; deriv.dy = dy;
    deriv.ddx = ddx; deriv.ddy = ddy;
end
