function Main_control_code()

    % Robot physical params
    r  = 0.04445;      % Wheel radius
    L  = 0.393;         % Wheelbase
    Ts = 0.15;          % Sample time

    wmax = 10;          % Wheel speed limit

    initState = [0; 0; 0];

    % Generate reference path
    [traj, deriv] = buildTrajectory(Ts);

    % Storage containers
    q_KES = initState;    vW_KES = [];   err_KES = [];
    q_FL  = initState;    vW_FL  = [];   err_FL  = [];

    % Controller gains
    gainsKES = [5, 5, 0.1, 0.1];
    gainsFL  = [1.5, 1.5, 0.5, 0.5];
    offset   = 0.1;

    N = numel(traj.t);

    for k = 1:N
        xr = traj.x(k);  yr = traj.y(k);
        dx = deriv.dx(k); dy = deriv.dy(k);
        ddx = deriv.ddx(k); ddy = deriv.ddy(k);

        % ---------- KES Controller ----------
        qk = q_KES(:, end);

        [vK, wK] = Trajectory_Tracking_law( ...
            qk(1), qk(2), qk(3), xr, yr, dx, dy, ddx, ddy, ...
            Ts, gainsKES(1), gainsKES(2), gainsKES(3), gainsKES(4), 1);

        [vK, wK] = unicycle_saturation(wmax, wmax, vK, wK, r, L);
        q_KES(:, end+1) = DiffDrive(qk, vK, wK, Ts);
        vW_KES(:, end+1) = [vK; wK];
        err_KES(end+1) = hypot(qk(1)-xr, qk(2)-yr);

        % ---------- FL-PD Controller ----------
        qf = q_FL(:, end);

        [vF, wF] = Trajectory_Tracking_law( ...
            qf(1), qf(2), qf(3), xr, yr, dx, dy, ddx, ddy, ...
            Ts, gainsFL(1), gainsFL(2), gainsFL(3), gainsFL(4), 2, offset);

        [vF, wF] = unicycle_saturation(wmax, wmax, vF, wF, r, L);
        q_FL(:, end+1) = DiffDrive(qf, vF, wF, Ts);
        vW_FL(:, end+1) = [vF; wF];
        err_FL(end+1) = hypot(qf(1)-xr, qf(2)-yr);
    end

    plotResults(q_KES, q_FL, vW_KES, vW_FL, err_KES, err_FL, traj);
end
