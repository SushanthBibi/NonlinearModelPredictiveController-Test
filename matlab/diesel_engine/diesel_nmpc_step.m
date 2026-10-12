function u = diesel_nmpc_step(x, t, mode)
%DIESEL_NMPC_STEP  One NMPC update (called every Ts seconds from Simulink).
%   u = diesel_nmpc_step(x, t, mode)
%   x    measured state [T_e; T_r; T_o]
%   t    simulation time [s] - used to read the route preview
%   mode 2 = NMPC active; anything else returns zeros (controller unused)
%
%   Optimal control problem, solved with fmincon (SQP) by single shooting:
%     min  sum_k  w_T (T_e - T_ref)^2 + w_To (T_o - T_o_ref)^2
%               + w_fuel * (P_fric + P_fan + P_pump)/eta_ind      [kW of fuel]
%               + w_slack * (soft-limit violations)^2
%          + w_du * ||delta u||^2
%     s.t. x_{k+1} = RK4(x_k, u_k, d_k)   (diesel_model.m)
%          u_pump in [u_pump_min, 1], u_fan, u_th in [0, 1]
%   The disturbance preview d_k (engine speed, brake power, vehicle speed,
%   ambient) comes from the planned route - this is what makes it predictive.
%   Inputs are move-blocked (cfg.blocks) to keep the problem small.
%
%   Requires the Optimization Toolbox (fmincon).

persistent S
u = zeros(3,1);
if mode < 1.5
    return
end
if isempty(S)
    S.P   = evalin('base', 'P');
    S.cyc = evalin('base', 'cyc');
    S.cfg = evalin('base', 'nmpc_cfg');
    S.u_prev = [S.P.u_pump_min; 0; 0];
    S.z = [];
    S.opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
        'MaxIterations', S.cfg.max_iter, 'MaxFunctionEvaluations', 4000, ...
        'OptimalityTolerance', 1e-4, 'StepTolerance', 1e-6);
end
P = S.P; cfg = S.cfg;
Np = cfg.Np; Nb = numel(cfg.blocks); dt = cfg.dt;
blk = repelem(1:Nb, cfg.blocks);              % step -> input block

% ---- route preview: interval mean (dynamics) and interval peak load (limits)
n = numel(S.cyc.t);
Dm = zeros(4, Np); Dpk = zeros(4, Np);
k0 = round(t) + 1;
for i = 1:Np
    idx = min(k0 + (i-1)*dt + (0:dt-1), n);
    Dm(:,i)  = mean(S.cyc.D(:, idx), 2);
    Dpk(:,i) = Dm(:,i);
    Dpk(2,i) = max(S.cyc.D(2, idx));
end

lb = repmat([P.u_pump_min; 0; 0], Nb, 1);
ub = ones(3*Nb, 1);
if isempty(S.z)
    z0 = repmat(S.u_prev, Nb, 1);
else
    z0 = S.z;                                  % warm start
end
x0 = x(:);
cost = @(z) nmpc_cost(z, x0, Dm, Dpk, S.u_prev, P, cfg, blk);
[z, ~, flag] = fmincon(cost, z0, [], [], [], [], lb, ub, [], S.opts);
if flag < 0
    z = z0;                                    % keep last plan if the solver failed
end
S.z = z;
U = reshape(z, 3, Nb);
u = min(max(U(:,1), lb(1:3)), 1);
S.u_prev = u;
end

function J = nmpc_cost(z, x, Dm, Dpk, u_prev, P, cfg, blk)
w = cfg.w;
U = reshape(z, 3, []);
J = 0;
for i = 1:cfg.Np
    ui = U(:, blk(i));
    [~, ypk0] = diesel_model(x, ui, Dpk(:,i), P);       % metal temp at interval start
    x = diesel_rk4(x, ui, Dm(:,i), cfg.dt, P);
    [~, y]    = diesel_model(x, ui, Dm(:,i), P);
    [~, ypk1] = diesel_model(x, ui, Dpk(:,i), P);       % ... and at interval end
    fuel_kW = (y.P_fric + y.P_fan + y.P_pump) / P.eta_ind / 1e3;
    viol = [max(0, x(1) - (P.T_e_max - cfg.margin));
            max(0, x(3) - (P.T_o_max - cfg.margin));
            max(0, ypk0.T_metal - (P.T_metal_max - cfg.margin));
            max(0, ypk1.T_metal - (P.T_metal_max - cfg.margin))];
    J = J + w.T*(x(1) - w.T_ref)^2 + w.To*(x(3) - w.T_o_ref)^2 ...
          + w.fuel*fuel_kW + w.slack*sum(viol.^2);
end
dU = [U(:,1) - u_prev, diff(U, 1, 2)];
J = J + sum(w.du .* sum(dU.^2, 2));
end
