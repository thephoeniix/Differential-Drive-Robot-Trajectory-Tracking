function [v_s, w_s] = unicycle_saturation(wrmax, wlmax, v, w, r, d)

    % Limit angular velocity
    w_lim = 2 * wrmax * r / d;
    w_s = max(min(w, w_lim), -w_lim);

    % Max linear velocity under wheel constraints
    v_max_r = r*wrmax - (d/2)*w_s;
    v_max_l = r*wlmax + (d/2)*w_s;

    v_upper = min(v_max_r, v_max_l);
    v_lower = -min(v_max_r, v_max_l);

    v_s = max(min(v, v_upper), v_lower);
end
