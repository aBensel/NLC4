% Clear workspace for better repeatability. Clear mex is required because acados cannot overwrite
% its generated MEX-functions otherwise
clear
clear mex

setup_acados();

%% Simulate the problem
[model, optim] = NLChain_4();
[x, u, solver_stats] = mpc_benchmark_NLChain(model, optim);

%% Plot resulting NLChain_4 trajectories

% Plot position trajectories
figure()
ax1 = subplot(431);
plot(0:model.dT:model.Tf, squeeze(x(1,1,:)))
xlabel('Time t in s')
ylabel('Position x(t)')
title('Node 1')
ax2 = subplot(432);
plot(0:model.dT:model.Tf, squeeze(x(2,1,:)))
xlabel('Time t in s')
ylabel('Position y(t)')
title('Node 1')
ax3 = subplot(433);
plot(0:model.dT:model.Tf, squeeze(x(3,1,:)))
xlabel('Time t in s')
ylabel('Position z(t)')
title('Node 1')
ax4 = subplot(434);
plot(0:model.dT:model.Tf, squeeze(x(4,1,:)))
xlabel('Time t in s')
ylabel('Position x(t)')
title('Node 2')
ax5 = subplot(435);
plot(0:model.dT:model.Tf, squeeze(x(5,1,:)))
xlabel('Time t in s')
ylabel('Position y(t)')
title('Node 2')
ax6 = subplot(436);
plot(0:model.dT:model.Tf, squeeze(x(6,1,:)))
xlabel('Time t in s')
ylabel('Position z(t)')
title('Node 2')
ax7 = subplot(437);
plot(0:model.dT:model.Tf, squeeze(x(7,1,:)))
xlabel('Time t in s')
ylabel('Position x(t)')
title('Node 3')
ax8 = subplot(438);
plot(0:model.dT:model.Tf, squeeze(x(8,1,:)))
xlabel('Time t in s')
ylabel('Position y(t)')
title('Node 3')
ax9 = subplot(439);
plot(0:model.dT:model.Tf, squeeze(x(9,1,:)))
xlabel('Time t in s')
ylabel('Position z(t)')
title('Node 3')
ax10 = subplot(4,3,10);
plot(0:model.dT:model.Tf, squeeze(x(10,1,:)))
xlabel('Time t in s')
ylabel('Position x(t)')
title('Node 4')
ax11 = subplot(4,3,11);
plot(0:model.dT:model.Tf, squeeze(x(11,1,:)))
xlabel('Time t in s')
ylabel('Position y(t)')
title('Node 4')
ax12 = subplot(4,3,12);
plot(0:model.dT:model.Tf, squeeze(x(12,1,:)))
xlabel('Time t in s')
ylabel('Position z(t)')
title('Node 4')
sgtitle('Position Trajectories')
linkaxes([ax1, ax2,  ax3, ax4, ax5, ax6, ax7, ax8, ax9, ax10, ax11, ax12], 'x')

% Plot velocity trajectories
figure()
ax13 = subplot(331);
plot(0:model.dT:model.Tf, squeeze(x(13,1,:)))
xlabel('Time t in s')
ylabel('Velocity x dot(t)')
title('Node 1')
ax14 = subplot(332);
plot(0:model.dT:model.Tf, squeeze(x(14,1,:)))
xlabel('Time t in s')
ylabel('Velocity y dot(t)')
title('Node 1')
ax15 = subplot(333);
plot(0:model.dT:model.Tf, squeeze(x(15,1,:)))
xlabel('Time t in s')
ylabel('Velocity z dot(t)')
title('Node 1')
ax16 = subplot(334);
plot(0:model.dT:model.Tf, squeeze(x(16,1,:)))
xlabel('Time t in s')
ylabel('Velocity x dot(t)')
title('Node 2')
ax17 = subplot(335);
plot(0:model.dT:model.Tf, squeeze(x(17,1,:)))
xlabel('Time t in s')
ylabel('Velocity y dot(t)')
title('Node 2')
ax18 = subplot(336);
plot(0:model.dT:model.Tf, squeeze(x(18,1,:)))
xlabel('Time t in s')
ylabel('Velocity z dot(t)')
title('Node 2')
ax19 = subplot(337);
plot(0:model.dT:model.Tf, squeeze(x(19,1,:)))
xlabel('Time t in s')
ylabel('Velocity x dot(t)')
title('Node 3')
ax20 = subplot(338);
plot(0:model.dT:model.Tf, squeeze(x(20,1,:)))
xlabel('Time t in s')
ylabel('Velocity y dot(t)')
title('Node 3')
ax21 = subplot(339);
plot(0:model.dT:model.Tf, squeeze(x(21,1,:)))
xlabel('Time t in s')
ylabel('Velocity z dot(t)')
title('Node 3')
sgtitle('Velocity Trajectories')
linkaxes([ax13, ax14,  ax15, ax16, ax17, ax18, ax19, ax20, ax21], 'x')

% Plot input trajectories
figure()
ax22 = subplot(311);
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(1,1,:)))
xlabel('Time t in s')
ylabel('Velocity u_x(t)')
title('Input in x direction')
ylim([-1.5 1.5])
ax23 = subplot(312);
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(2,1,:)))
xlabel('Time t in s')
ylabel('Velocity u_y(t)')
title('Input in y direction')
ylim([-1.5 1.5])
ax24 = subplot(313);
stairs(0:model.dT:model.Tf-model.dT, squeeze(u(3,1,:)))
xlabel('Time t in s')
ylabel('Velocity u_z(t)')
title('Input in z direction')
ylim([-1.5 1.5])
sgtitle('Input Signal')
linkaxes([ax22, ax23,  ax24], 'x')

%% visualize chain

figure()
pause(0.1)
for k = 1:(model.Tf/model.dT)

% arrange position states
X_p = [0; x(1:3:10,:,k)];
Z_p = [0; x(3:3:12,:,k)];
    
plot(X_p, Z_p, '-o', 'Color', 'r', 'MarkerSize', 10, 'MarkerFaceColor', 'auto')
grid on
title('Movement of the chain')
xlabel('x Coordinate')
ylabel('z Coordinate')
xlim([0, 9])
ylim([-12, 2])

drawnow limitrate

end