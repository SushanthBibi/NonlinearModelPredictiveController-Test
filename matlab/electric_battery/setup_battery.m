%SETUP_BATTERY  Load parameters, mission and NMPC settings into the workspace.
%   Choose the climate before running, e.g.  scenario = 'winter';
if ~exist('scenario', 'var'),  scenario  = 'summer'; end   % 'summer' | 'mild' | 'winter'
if ~exist('ctrl_mode', 'var'), ctrl_mode = 2; end          % 1 = rule-based BMS, 2 = NMPC
if ~exist('use_reality_gap', 'var'), use_reality_gap = true; end

switch scenario
    case 'summer', T_ambient = 42; T_init = 38;   % Vellore summer, pack parked hot
    case 'mild',   T_ambient = 22; T_init = 24;
    case 'winter', T_ambient = -5; T_init = 4;
    otherwise, error('scenario must be summer, mild or winter');
end

[P, Gap] = battery_params();
if ~use_reality_gap
    Gap.plate_factor = 1; Gap.plate_temp_coeff = 0; Gap.chiller_amb_coeff = 0; Gap.edrive_leak = 0;
end
mission = battery_mission(T_ambient);

% flat copies used by the Fcn blocks: b_<name> (physics), g_<name> (reality gap)
fn = fieldnames(P);
for i_ = 1:numel(fn), eval(sprintf('b_%s = P.%s;', fn{i_}, fn{i_})); end
fn = fieldnames(Gap);
for i_ = 1:numel(fn), eval(sprintf('g_%s = Gap.%s;', fn{i_}, fn{i_})); end
clear fn i_

dist  = [mission.t, mission.D'];            % From Workspace: [t P_drive P_chg_max v T_amb]
Tstop = mission.t(end);
x0    = [T_init; T_init; T_init; 0.50];     % [T_c T_s T_w SOC]

Ts_twin = 60;                               % twin / NMPC update interval [s]
noise_std = [0.1; 0.1; 0.003];              % sensor noise: T_s, T_w [K], SOC [-]

twin_cfg.dt       = Ts_twin;
twin_cfg.Np       = 40;                     % 40 x 60 s = 40-min look-ahead
twin_cfg.blocks   = [1 2 3 4 6 8 8 8];      % move blocking (sums to Np)
twin_cfg.max_iter = 40;
twin_cfg.Q_ekf    = [2e-3 2e-3 2e-3 1e-8];  % process noise per second
twin_cfg.R_ekf    = noise_std'.^2;
twin_cfg.w.energy = 1.0;    % per kW of cooling/heating power
twin_cfg.w.charge = 0.25;   % reward per kW of charge power accepted
twin_cfg.w.age    = 2.0;    % per unit Arrhenius ageing rate
twin_cfg.w.band   = 2.0;    % per K^2 outside 18-38 degC
twin_cfg.w.limit  = 1e3;    % per K^2 above T_max - 2
twin_cfg.w.du     = 2.0;
twin_cfg.w.fight  = 50;     % chiller and heater at the same time
