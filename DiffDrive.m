function q_next = DiffDrive(state, v, w, Ts)
% Integrates differential-drive kinematics using ODE45

    if numel(state) ~= 3
        error('State must be [x; y; theta]');
    end

    f = @(t, s) [ v*cos(s(3));
                  v*sin(s(3));
                  w ];

    opts = odeset('RelTol',1e-6, 'AbsTol',1e-8);
    [~, S] = ode45(f, [0 Ts], state(:), opts);

    q_next = S(end, :)';
    q_next(3) = wrapToPi(q_next(3));
end
