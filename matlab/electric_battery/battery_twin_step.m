function [u, xhat] = battery_twin_step(y, u_applied, t, mode)
%BATTERY_TWIN_STEP  Digital twin update + NMPC, called every 60 s from Simulink.
%   [u, xhat] = battery_twin_step(y, u_applied, t, mode)
%   y          measured [T_s; T_w; SOC] (noisy)
%   u_applied  actuator commands applied over the last interval
%   t          simulation time [s] (reads the route preview)
%   mode       2 = NMPC active; otherwise only the twin (EKF) runs and u = 0
%   xhat       twin state estimate [T_c; T_s; T_w; SOC] - T_c is not measured
%
%   1) Extended Kalman filter keeps the physics twin in sync with the sensors
%      and estimates the cell-core temperature (virtual sensor).
%   2) NMPC (fmincon, single shooting, move blocking) over a 40-min horizon:
%        min sum_k  w_energy*P_thermal - w_charge*P_chg + w_age*ageing
%                 + w_band*(outside 18-38 degC)^2 + w_limit*(above T_max-2)^2
%             + w_du*||delta u||^2 + chiller/heater-fighting penalty
%      using the route preview (traction power, charger power, speed, ambient),
%      so it can pre-cool or pre-heat before the charging stop.
%   Requires the Optimization Toolbox (fmincon).

persistent S
if isempty(S)
    S.P   = evalin('base', 'P');
    S.M   = evalin('base', 'mission');
    S.cfg = evalin('base', 'twin_cfg');
    S.x   = [y(1); y(1); y(2); y(3)];          % core unknown: start at surface temp
    S.Pc  = diag([25, 1, 1, 1e-4]);
    S.t_last = t;
    S.u_prev = [0.3; 0; 0; 0; 0];
    S.z = [];
    S.opts = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', ...
        'MaxIterations', S.cfg.max_iter, 'MaxFunctionEvaluations', 5000, ...
        'OptimalityTolerance', 1e-3, 'StepTolerance', 1e-5);
end
P = S.P; cfg = S.cfg; n = numel(S.M.t);

% ---------------- 1) EKF ----------------
dt = t - S.t_last;
if dt > 0.5
    idx = min(round(S.t_last) + 1 : round(t), n);
    d_mean = mean(S.M.D(:, idx), 2);
    f = @(xx) battery_rk4(battery_rk4(xx, u_applied(:), d_mean, dt/2, P), u_applied(:), d_mean, dt/2, P);
    x_pred = f(S.x);
    A = zeros(4);
    h = [1e-3 1e-3 1e-3 1e-6];
    for i = 1:4                                % finite-difference Jacobian
        e = zeros(4,1); e(i) = h(i);
        A(:,i) = (f(S.x + e) - x_pred) / h(i);
    end
    S.x  = x_pred;
    S.Pc = A*S.Pc*A' + diag(cfg.Q_ekf)*dt;
end
C = [0 1 0 0; 0 0 1 0; 0 0 0 1];
K = S.Pc*C' / (C*S.Pc*C' + diag(cfg.R_ekf));
S.x  = S.x + K*(y(:) - C*S.x);
S.Pc = (eye(4) - K*C)*S.Pc;
S.t_last = t;
xhat = S.x;

% ---------------- 2) NMPC ----------------
u = zeros(5,1);
if mode < 1.5
    return
end
Np = cfg.Np; Nb = numel(cfg.blocks); Ts = cfg.dt;
blk = repelem(1:Nb, cfg.blocks);
D = zeros(4, Np);
k0 = round(t) + 1;
for i = 1:Np                                   % interval-mean route preview
    idx = min(k0 + (i-1)*Ts + (0:Ts-1), n);
    D(:,i) = mean(S.M.D(:, idx), 2);
end
lb = repmat([P.u_pump_min; 0; 0; 0; 0], Nb, 1);
ub = ones(5*Nb, 1);
if isempty(S.z), z0 = repmat(max(S.u_prev, lb(1:5)), Nb, 1); else, z0 = S.z; end
cost = @(z) nmpc_cost(z, S.x, D, S.u_prev, P, cfg, blk);
[z, ~, flag] = fmincon(cost, z0, [], [], [], [], lb, ub, [], S.opts);
if flag < 0, z = z0; end
S.z = z;
U = reshape(z, 5, Nb);
u = min(max(U(:,1), lb(1:5)), 1);
S.u_prev = u;
end

function J = nmpc_cost(z, x, D, u_prev, P, cfg, blk)
w = cfg.w;
U = reshape(z, 5, []);
J = 0;
for i = 1:cfg.Np
    ui = U(:, blk(i));
    [~, yo] = battery_model(x, ui, D(:,i), P);
    J = J + w.energy*yo.P_thermal/1e3 - w.charge*yo.P_chg/1e3 + w.age*yo.age_rate;
    x = battery_rk4(x, ui, D(:,i), cfg.dt, P);
    Tc = x(1);
    J = J + w.band*(max(0, Tc - P.T_band_hi)^2 + 0.2*max(0, P.T_band_lo - Tc)^2) ...
          + w.limit*max(0, Tc - (P.T_max - 2))^2;
end
dU = [U(:,1) - u_prev, diff(U, 1, 2)];
J = J + w.du*sum(dU(:).^2) + w.fight*sum(U(2,:).*U(5,:));   % chiller x heater
end
