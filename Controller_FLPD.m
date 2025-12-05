function [v_cmd, w_cmd] = Controller_FLPD( ...
        px, py, ang, xr, yr, hdg_r, v_r, w_r, Ts, ...
        kp1, kp2, kd1, kd2, b)

    % Forward point of robot
    zr = [px + b*cos(ang);
          py + b*sin(ang)];

    % Forward reference point
    zref = [xr + b*cos(hdg_r);
            yr + b*sin(hdg_r)];

    % Jacobian matrix
    T = [cos(ang) -b*sin(ang);
         sin(ang)  b*cos(ang)];

    % Reference velocity of forward point
    zref_dot = [
        v_r*cos(hdg_r) - b*sin(hdg_r)*w_r;
        v_r*sin(hdg_r) + b*cos(hdg_r)*w_r
    ];

    % Error
    e = zr - zref;

    % Numerical derivative
    persistent prev_e
    if isempty(prev_e), prev_e = e; end
    e_dot = (e - prev_e) / Ts;
    prev_e = e;

    % PD correction
    Kp = diag([kp1 kp2]);
    Kd = diag([kd1 kd2]);
    u = -Kp * e - Kd * e_dot + zref_dot;

    % Solve input mapping
    if abs(det(T)) > 1e-6
        vw = T \ u;
    else
        vw = pinv(T) * u;
    end

    v_cmd = vw(1);
    w_cmd = vw(2);
end
