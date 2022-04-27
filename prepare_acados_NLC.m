function ocp = prepare_acados_NLC(model, optim)
%PREPARE_ACADOS Prepare the acados optimizer for the respective model
%
%   model -> Struct describing the model to implement a controller for
%   optim -> Struct with optimization parameters
%
%   ocp   -> Matlab object representing the prepared acados optimizer

% Calculate prediction horizon in seconds
ocp_N = double(optim.N);
ocp_T = ocp_N * model.dT;

%% Define cost function
% The cost function on ACADOS is defined in terms of a virtual output y. In
% our case, we stack the states and inputs together to form y.

nx = model.nx;
nu = model.nu;
ny = nx + nu;                   % Dimension of the virtual output
Vx = [ eye(nx); zeros(nu,nx) ]; % Mapping from states to virtual output
Vu = [ zeros(nx,nu); eye(nu) ]; % Mapping from inputs to virtual output

% lower and upper bounds on the input
u_ub = optim.b(1:nu);
u_lb = - optim.b(nu+1:2*nu);

% set up reference to be tracked
y_ref = [model.xdes; model.udes];

% Weighting matrix for the virtual output
W  = blkdiag(optim.Q, optim.R); 

% set up terminal penalty
Vx_e = eye(nx); %Vx
W_e = optim.P; %blkdiag(optim.P, zeros(size(optim.R)));
y_ref_e = model.xdes; %y_ref;


%% Build ACADOS OCP model
% For details on the meaning of each of these terms, check the ACADOS documentation at 
% https://github.com/acados/acados/blob/master/docs/problem_formulation/problem_formulation_ocp_mex.pdf

ocp_model = acados_ocp_model();
ocp_model.set('name', model.name);
ocp_model.set('T', ocp_T);

% Set dimension of the decision variables
ocp_model.set('dim_nx', nx);
ocp_model.set('dim_nu', nu);
ocp_model.set('dim_ny', ny);
ocp_model.set('dim_ny_e', 0); % No additional terminal constraint. See also the comment further down
ocp_model.set('dim_nz', 0);   % No algebraic variables

% Set symbolic variables
sym_x = casadi.SX.sym('x', nx);
sym_u = casadi.SX.sym('u', nu);
ocp_model.set('sym_x', sym_x);
ocp_model.set('sym_u', sym_u);

% Define dynamic properties of the model
ocp_model.set('dyn_type', 'explicit'); % Use explicit continuous-time ODE as model
ocp_model.set('dyn_expr_f', model.ode(sym_x, sym_u));

% Set up cost function
ocp_model.set('cost_type', 'linear_ls');  % Linear least-squares cost function
ocp_model.set('cost_Vu', Vu);
ocp_model.set('cost_Vx', Vx);
ocp_model.set('cost_Vz', zeros(ny,0));    % No algebraic variables, so setting to 0
ocp_model.set('cost_W', W);
ocp_model.set('cost_y_ref', y_ref); % in this case we are considering a reference

% Set up input constraints
% ocp_model.set('constr_C', zeros(nu, nx));  % No constraints on the states
% ocp_model.set('constr_D', eye(nu));
% ocp_model.set('constr_lg', u_lb);       % Lower and upper limit on the control
% ocp_model.set('constr_ug', u_ub);
ocp_model.set('constr_Jbu', eye(nu));
ocp_model.set('constr_lbu', u_lb);
ocp_model.set('constr_ubu', u_ub);

% Use additional terminal cost. 
ocp_model.set('cost_type_e', 'linear_ls');
ocp_model.set('cost_Vx_e', Vx_e); % for terminal cost
ocp_model.set('cost_W_e', W_e); % for terminal cost
ocp_model.set('cost_y_ref_e', y_ref_e); % for terminal cost

% Temporarily set initial conditions, without this the model contruction fails. Will be overwritten
% before the simulation anyway.
ocp_model.set('constr_x0', model.x0);

%% Configure ACADOS OCP solver
ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N', ocp_N);

% Set up the internal simulator.
ocp_opts.set('sim_method', 'erk');
ocp_opts.set('sim_method_num_stages', 4);
ocp_opts.set('sim_method_num_steps', 2);

% Set up the NLP solver. If the solver is only allowed to perform a single iteration, switch to
% real-time iteration mode.
if optim.iter_max == 1
    ocp_opts.set('nlp_solver', 'sqp_rti');
else
    ocp_opts.set('nlp_solver', 'sqp');
end
ocp_opts.set('nlp_solver_max_iter', optim.iter_max);

% Set up the QP solver
ocp_opts.set('qp_solver', 'partial_condensing_hpipm');
ocp_opts.set('qp_solver_cond_N', ocp_N);

%% Generate OCP
ocp = acados_ocp(ocp_model, ocp_opts);
end
