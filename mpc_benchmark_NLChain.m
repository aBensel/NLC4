function [x_traj, u_traj, solver_stats] = mpc_benchmark_NLChain(model, optim)
%MPC_BENCHMARK Run the benchmark scenario

nx = model.nx;
nu = model.nu;

% Prepare solvers from acados and CasADi / Ipopt
ocp_acados = prepare_acados_NLC(model, optim);

% set number of runs
averaging = 1;

%% Reserve memory for saving the simulation results
steps = floor(model.Tf/model.dT);

% Reserve memory for simulated trajectories
x_traj          = zeros(nx, averaging, steps);
x_traj(:,:,1) = repmat(model.x0, 1, averaging);
u_traj          = zeros(nu, averaging, steps);

% Prepare time statistics
solver_stats = struct('total', zeros(averaging, steps),...
                      'solv',  zeros(averaging, steps),...
                      'prep',  zeros(averaging, steps),...
                      'iter',  zeros(averaging, steps),...
                      'status', zeros(averaging, steps));

%% Simulate the problem
for j = 1:averaging
    for k = 1:steps
        % Run the acados based solver
        ocp_acados.set('constr_x0', x_traj(:,j,k));
        %ocp_acados.set('init_x', [x_traj(:,j,k); zeros(N*nx,1)]);
        ocp_acados.solve();
        solver_stats.total(j,k) = ocp_acados.get('time_tot');
        solver_stats.iter(j,k)  = ocp_acados.get('sqp_iter');
        solver_stats.prep(j,k)  = ocp_acados.get('time_lin')...
                + ocp_acados.get('time_reg') + ocp_acados.get('time_sim');
        solver_stats.solv(j,k)  = ocp_acados.get('time_qp_sol');
        solver_stats.status(j,k) = ocp_acados.get('status');
        u_traj(:,j,k) = ocp_acados.get('u', 0);

        
        % Simulate continuous-time system
        odeFun = @(~,y) model.ode(y, u_traj(:,j,k));
        [~,y] = ode45(odeFun, [0, model.dT]+(k-1)*model.dT, x_traj(:,j,k));
        x_traj(:,j,k+1) = y(end, :);
    end
end
end


