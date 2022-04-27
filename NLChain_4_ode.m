function dxdt = NLChain_4_ode(x, u)
%NLChain_4_ODE Implementation of the ODE for the NLChain_4
%   This function implements the ODE for the NLChain_4. This function can also be used to define the
%   symbolic expression required for both the acados & CasADi / Ipopt solvers.

%% Define constants
L = 0.55; % length between two nodes
g = 9.81; % gravitation
k = 0.1; % spring stiffness between nodes
m = 0.45; % mass of nodes
n = 4; % number of nodes without the one at the origin

AN_12 = AdjNorm(x, 1, 2, L);
AN_23 = AdjNorm(x, 2, 3, L);
AN_34 = AdjNorm(x, 3, 4, L);
AN_45 = AdjNorm(x, 4, 5, L);

aux1 = n*k/m * (-2*n + AN_12 + AN_23);
aux2 = n*k/m * (n - AN_23);
aux3 = n*k/m * (-2*n + AN_23 + AN_34);
aux4 = n*k/m * (n - AN_34);
aux5 = n*k/m * (-2*n + AN_34 + AN_45);
aux6 = n*k/m * (n - AN_45);

%% Equations of motion
dxdt = [x(13);
        x(14);
        x(15);
        x(16);
        x(17);
        x(18);
        x(19);
        x(20);
        x(21);
        u(1);
        u(2);
        u(3);
        aux1 * x(1) + aux2 * x(4);
        aux1 * x(2) + aux2 * x(5);
        aux1 * x(3) + aux2 * x(6) - g;
        aux2 * x(1) + aux3 * x(4) + aux4 * x(7);
        aux2 * x(2) + aux3 * x(5) + aux4 * x(8);
        aux2 * x(3) + aux3 * x(6) + aux4 * x(9) - g;
        aux4 * x(4) + aux5 * x(7) + aux6 * x(10);
        aux4 * x(5) + aux5 * x(8) + aux6 * x(11);
        aux4 * x(6) + aux5 * x(9) + aux6 * x(12) - g];

end


function AN = AdjNorm(x, p1, p2, L)
% This function computes an essential part of the nonlinearities involved
% in the ODE

% inputs:
% x - state vector
% p1 - integer index of which position p1 in chain
% p2 - integer index of which position p2 in chain
% L - Length between two nodes

% output:
% AN = L/norm(p2 - p1)

if p1 == 1
    q1 = [0; 0; 0];
else
    q1 = [x(1 + 3*(p1-2));
          x(2 + 3*(p1-2));
          x(3 + 3*(p1-2));];
end

if p2 == 1
    q2 = [0; 0; 0];
else
    q2 = [x(1 + 3*(p2-2));
          x(2 + 3*(p2-2));
          x(3 + 3*(p2-2));];
end

AN = L/norm(q2-q1);

end