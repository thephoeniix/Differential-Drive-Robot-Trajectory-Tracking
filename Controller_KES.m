function [v_out, w_out] = Controller_KES( ...
        px, py, ang, xr, yr, dxr, dyr, ddxr, ddyr, ...
        Ts, kp_pos, kp_ang, kd_pos, kd_ang)

    % Position error in global frame
    e_xy = [xr - px; yr - py];

    % Transform to robot frame
    R = [cos(ang) sin(ang); -sin(ang) cos(ang)];
    e_r = R * e_xy;

    ex = e_r(1);
    ey = e_r(2);

    % Orientation error
    hdg_ref = atan2(dyr, dxr);
    e_theta = wrapToPi(hdg_ref - ang);

    % Reference velocities
    v_ref = hypot(dxr, dyr);
    denom = max(dxr^2 + dyr^2, 1e-8);
    w_ref = (ddyr*dxr - ddxr*dyr) / denom;

    % Linear velocity
    v_out = v_ref * cos(e_theta) + kp_pos * ex;

    % Angular correction
    if abs(e_theta) < 1e-6
        s_term = 1;
    else
        s_term = sin(e_theta) / e_theta;
    end

    w_out = w_ref + kp_ang * v_ref * s_term * ey + kd_ang * e_theta;

    % Derivative term for ex
    persistent prev_ex
    if isempty(prev_ex), prev_ex = ex; end

    if kd_pos > 0
        ex_dot = (ex - prev_ex) / Ts;
        v_out = v_out + kd_pos * ex_dot;
        prev_ex = ex;
    end
end
