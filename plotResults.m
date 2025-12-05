function plotResults(qK, qF, vK, vF, eK, eF, traj)

    figure;
    subplot(1,2,1)
    hold on; grid on; axis equal;
    plot(traj.x, traj.y, 'r--');
    plot(qK(1,:), qK(2,:), 'b');
    plot(qF(1,:), qF(2,:), 'm');
    legend('Reference','KES','FL-PD')
    title('Trajectory Comparison')

    subplot(1,2,2)
    hold on; grid on;
    plot(eK, 'b'); 
    plot(eF, 'm');
    title('Tracking Error')
    legend('KES','FL-PD')
end
