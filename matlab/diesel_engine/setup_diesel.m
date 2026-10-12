%SETUP_DIESEL  Load parameters, drive cycle and NMPC settings into the workspace.
%   Every Simulink block reads its values from here. Run before simulating.
if ~exist('T_ambient', 'var'), T_ambient = 25; end   % [degC]
if ~exist('ctrl_mode', 'var'), ctrl_mode = 2;  end   % 1 = conventional, 2 = NMPC

P   = diesel_params();
cyc = diesel_drive_cycle(T_ambient);

% flat copies (p_<name>) of every parameter, used by the Fcn blocks
fn = fieldnames(P);
for i_ = 1:numel(fn), eval(sprintf('p_%s = P.%s;', fn{i_}, fn{i_})); end
clear fn i_

dist  = [cyc.t, cyc.D'];                     % From Workspace: [t N P_b v T_amb]
Tstop = cyc.t(end);
x0    = T_ambient*[1; 1; 1];                 % cold start

nmpc_cfg.dt       = 5;                       % control interval [s]
nmpc_cfg.Np       = 24;                      % horizon steps (24 x 5 s = 2 min)
nmpc_cfg.blocks   = [2 2 4 4 6 6];           % move blocking (sums to Np)
nmpc_cfg.margin   = 1.0;                     % back-off from hard limits [K]
nmpc_cfg.max_iter = 60;
nmpc_cfg.w.T_ref  = 92;   nmpc_cfg.w.T   = 0.05;
nmpc_cfg.w.T_o_ref = 100; nmpc_cfg.w.To  = 0.01;
nmpc_cfg.w.fuel   = 1.0;
nmpc_cfg.w.slack  = 1e3;
nmpc_cfg.w.du     = 10*[1; 1; 1];
Ts_mpc = nmpc_cfg.dt;
