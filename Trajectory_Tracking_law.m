function [v_cmd, w_cmd] = Trajectory_Tracking_law( ...
        px, py, ang, xr, yr, dxr, dyr, ddxr, ddyr, ...
        Ts, kp_pos, kp_ang, kd_pos, kd_ang, modeSel, offset)

    if nargin < 16
        offset = 0.1;
    end
    
    switch modeSel
        case 1
            [v_cmd, w_cmd] = Controller_KES( ...
                px, py, ang, xr, yr, dxr, dyr, ddxr, ddyr, ...
                Ts, kp_pos, kp_ang, kd_pos, kd_ang);

        case 2
            hdg = atan2(dyr, dxr);
            v_ref = hypot(dxr, dyr);
            w_ref = (ddyr*dxr - ddxr*dyr) / max((dxr^2 + dyr^2), 1e-6);

            [v_cmd, w_cmd] = Controller_FLPD( ...
                px, py, ang, xr, yr, hdg, v_ref, w_ref, Ts, ...
                kp_pos, kp_ang, kd_pos, kd_ang, offset);

        otherwise
            error('Invalid controller mode');
    end
end
