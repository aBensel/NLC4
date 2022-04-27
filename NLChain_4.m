function [model, optim] = NLChain_4()
% Defines the NLC4 model

nx = 21; % Number of states
nu = 3; % Number of inputs
n = 4; % number of point masses without the one in the origin

%% Define simulation scenario
x0 = [7.5 * 1/n; 0; 0;
      7.5 * 2/n; 0; 0;
      7.5 * 3/n; 0; 0;
      7.5 * 4/n; 0; 0;
      0; 0; 0;
      0; 0; 0;
      0; 0; 0];% Initial condition

%x0 = x0 + 0.1 * ones(21, 1);

xdes = [zeros(9, 1);
        7.5;
        zeros(11,1)]; % desired states / xdes for position states is arbitrary because there is no weight on them (except for final node)
u0 = [0; 0; 0];            % initial input
udes = [0; 0; 0];          % desired input
dT = 0.1;              % Sampling time of the controller and the underlying discrete-time system
Tf = 50;                 % Final simulation time of this scenario

% Define inequality constraints subject to: A*u <= b
% if empty, set to []
A = [eye(nu);
    -eye(nu)];
b = [1; 1; 1; 1; 1; 1];

% Define equality constraints subject to: Aeq*u = beq
% if empty set to []
Aeq = [];
beq = [];

%% Define solver and cost function parameters
N        = int32(80); % Prediction horizon in steps
iter_max = int32(4); % Maximum number of iterations before the solver should terminate
tol_stat = 1e-2;      % Tolerance on the stationarity gradient. If the gradient is less than this
                      % number, the qLMPC solver terminates

% We use a simple quadratic stage cost function in the form of
% V_k(x,u) = (x(k)-xdes)'*Q*(x(k)-xdes) + (u(k)-udes)'*R*(u(k)-udes)

w_v = 25; % weight on velocities
w_x = 2.5; % weight on the position of the final node (which is controlled via the input)
w_u = 0.1; % weight on the input
w_t = 10; % terminal cost on the position of the final node

Q = [zeros(3 * (n-1), nx);
     zeros(3, (nx-3)/2), w_x * eye(3), zeros(3, (nx-3)/2);
     zeros(3 * (n-1), nx - 3 * (n-1)), w_v * eye(3 * (n-1))];

R = w_u * eye(3);

P = [zeros(3 * (n-1), nx);
     zeros(3, 3 * (n-1)), w_t * eye(3), zeros(3, 3 * (n-1));
     zeros(3 * (n-1), nx)];

%% Collect data into structures

% Collect in optimizer settings
optim = struct;
optim.N        = N;
optim.Q        = Q;
optim.R        = R;
optim.P        = P;
optim.iter_max = iter_max;
optim.tol_stat = tol_stat;
optim.A        = A;
optim.b        = b;
optim.Aeq      = Aeq;
optim.beq      = beq;

% Collect all other model specific values
model      = struct;
model.name = 'NLChain_4';
model.nx   = nx;
model.nu   = nu;
model.x0   = x0;
model.u0   = u0;
model.xdes = xdes;
model.udes = udes;
model.dT   = dT;
model.Tf   = Tf;
model.ode  = @NLChain_4_ode;

end
