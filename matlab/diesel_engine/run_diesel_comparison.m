%RUN_DIESEL_COMPARISON  Build the Simulink model, simulate the conventional
%cooling system and the NMPC on the same drive cycle, report KPIs and plot.
%
%   >> run_diesel_comparison
%
%   Optional before running:  T_ambient = 38;   (hot day, default 25 degC)
%   The NMPC run takes a few minutes (one fmincon solve every 5 s of a
%   36-min cycle).

clear diesel_nmpc_step
setup_diesel;
mdl = 'diesel_cooling_nmpc';
if ~exist([mdl '.slx'], 'file')
    build_diesel_model(mdl);
end
load_system(mdl);

names = {'Conventional', 'NMPC'};
R = struct();
for mode_ = 1:2
    ctrl_mode = mode_;                      %#ok<NASGU>  read by the model
    clear diesel_nmpc_step
    fprintf('Simulating %s ...\n', names{mode_});
    tic;
    out = sim(mdl);
    fprintf('  done in %.0f s\n', toc);
    R(mode_).x  = out.log_x;                % [T_e T_r T_o]
    R(mode_).u  = out.log_u;                % [u_pump u_fan u_th]
    R(mode_).pl = out.log_plant;            % [P_fan P_pump P_fric mdot_fuel T_metal]
end

% ---------------- KPIs ----------------
t = (0:size(R(1).x, 1) - 1)';
kpi_names = {'warm-up time to 85 degC [s]', 'coolant RMS dev. from 92 degC (warm) [K]', ...
    'peak coolant temp [degC]', 'peak oil temp [degC]', 'peak liner metal temp [degC]', ...
    'fan energy [kWh]', 'pump energy [kWh]', 'cooling-system energy [kWh]', ...
    'friction energy [kWh]', 'fuel consumed [kg]', 'time above coolant limit [s]'};
K = zeros(numel(kpi_names), 2);
for m_ = 1:2
    Te = R(m_).x(:,1); To = R(m_).x(:,3); pl = R(m_).pl;
    iw = find(Te >= 85, 1);
    if isempty(iw), iw = numel(Te); end
    K(:, m_) = [t(iw); sqrt(mean((Te(iw:end) - 92).^2)); max(Te); max(To); max(pl(:,5)); ...
        sum(pl(:,1))/3.6e6; sum(pl(:,2))/3.6e6; sum(pl(:,1) + pl(:,2))/3.6e6; ...
        sum(pl(:,3))/3.6e6; sum(pl(:,4)); sum(Te > P.T_e_max)];
end
fprintf('\n%-42s %14s %14s %10s\n', 'KPI', names{:}, 'change');
for i_ = 1:numel(kpi_names)
    ch_ = 100*(K(i_,2) - K(i_,1))/max(abs(K(i_,1)), eps);
    fprintf('%-42s %14.3f %14.3f %+9.1f%%\n', kpi_names{i_}, K(i_,1), K(i_,2), ch_);
end

% ---------------- plots ----------------
tm = t/60; c = lines(2);
figure('Name', 'Diesel cooling: conventional vs NMPC', 'Color', 'w');
subplot(5,1,1);
yyaxis left;  plot(tm, cyc.v(1:numel(tm))*3.6, 'k'); ylabel('speed [km/h]');
yyaxis right; plot(tm, cyc.P_b(1:numel(tm))/1e3); ylabel('brake power [kW]');
title('Drive cycle'); grid on
subplot(5,1,2); hold on
for m_ = 1:2, plot(tm, R(m_).x(:,1), 'Color', c(m_,:)); end
yline(P.T_e_max, 'k:'); ylabel('T_e [degC]'); legend(names{:}, 'limit'); grid on
subplot(5,1,3); hold on
for m_ = 1:2, plot(tm, R(m_).x(:,3), 'Color', c(m_,:)); end
ylabel('T_o [degC]'); grid on
subplot(5,1,4); hold on
for m_ = 1:2
    plot(tm, R(m_).u(:,2), '-', 'Color', c(m_,:));
    plot(tm, R(m_).u(:,1), '--', 'Color', c(m_,:));
end
ylabel('fan (solid), pump (dashed)'); grid on
subplot(5,1,5); hold on
for m_ = 1:2, plot(tm, (R(m_).pl(:,1) + R(m_).pl(:,2))/1e3, 'Color', c(m_,:)); end
ylabel('fan + pump [kW]'); xlabel('time [min]'); grid on
