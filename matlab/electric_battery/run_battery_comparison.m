%RUN_BATTERY_COMPARISON  Build the Simulink model, simulate the rule-based BMS
%and the digital-twin NMPC on the same mission, report KPIs and plot.
%
%   >> scenario = 'summer';   % or 'mild' / 'winter' (optional, default summer)
%   >> run_battery_comparison
%
%   The NMPC run takes several minutes (one fmincon solve per simulated minute
%   of a 2.5-hour mission).

clear battery_twin_step
setup_battery;
mdl = 'battery_thermal_twin';
if ~exist([mdl '.slx'], 'file')
    build_battery_model(mdl);
end
load_system(mdl);

names = {'Rule-based BMS', 'Twin + NMPC'};
R = struct();
for mode_ = 1:2
    ctrl_mode = mode_;                      %#ok<NASGU>  read by the model
    clear battery_twin_step
    fprintf('Simulating %s (%s) ...\n', names{mode_}, scenario);
    tic;
    out = sim(mdl);
    fprintf('  done in %.0f s\n', toc);
    R(mode_).x    = out.log_x;              % [T_c T_s T_w SOC]
    R(mode_).xhat = out.log_xhat;           % twin estimate
    R(mode_).u    = out.log_u;              % [pump chiller fan valve heater]
    R(mode_).pl   = out.log_plant;          % [P_thermal P_comp P_heat P_chg age_rate Q_gen]
end

% ---------------- KPIs ----------------
t = (0:size(R(1).x, 1) - 1)';
in_chg = t >= mission.charge_window(1) & t <= mission.charge_window(2);
i_end  = find(in_chg, 1, 'last');
kpi_names = {'thermal-system energy [kWh]', '  chiller compressor [kWh]', '  heater [kWh]', ...
    'energy charged in 30-min stop [kWh]', 'SOC after charging [%]', 'peak core temp [degC]', ...
    'time core > 40 degC [min]', 'ageing index (25 degC = 1)', 'core-temp estimate RMSE [K]'};
K = zeros(numel(kpi_names), 2);
for m_ = 1:2
    x = R(m_).x; pl = R(m_).pl;
    e = R(m_).xhat(600:end, 1) - x(600:end, 1);
    K(:, m_) = [sum(pl(:,1))/3.6e6; sum(pl(:,2))/3.6e6; sum(pl(:,3))/3.6e6; ...
        sum(pl(in_chg, 4))/3.6e6; 100*x(i_end, 4); max(x(:,1)); ...
        sum(x(:,1) > 40)/60; mean(pl(:,5)); sqrt(mean(e.^2))];
end
fprintf('\nScenario: %s, ambient %g degC\n', scenario, T_ambient);
fprintf('%-40s %16s %16s %10s\n', 'KPI', names{:}, 'change');
for i_ = 1:numel(kpi_names)
    ch_ = 100*(K(i_,2) - K(i_,1))/max(abs(K(i_,1)), eps);
    fprintf('%-40s %16.3f %16.3f %+9.1f%%\n', kpi_names{i_}, K(i_,1), K(i_,2), ch_);
end

% ---------------- plots ----------------
tm = t/60; c = lines(2); cw = mission.charge_window/60;
figure('Name', ['Battery thermal twin: ' scenario], 'Color', 'w');
subplot(5,1,1);
plot(tm, mission.P_drive(1:numel(tm))/1e3, 'k'); hold on
plot(tm, mission.P_chg_max(1:numel(tm))/1e3, 'r--');
xline(cw, '--'); ylabel('kW'); legend('traction', 'charger'); title('Mission'); grid on
subplot(5,1,2); hold on
for m_ = 1:2, plot(tm, R(m_).x(:,1), 'Color', c(m_,:)); end
plot(tm, R(2).xhat(:,1), ':', 'Color', c(2,:));
yline(P.T_band_hi, 'k:'); xline(cw, '--');
ylabel('core T_c [degC]'); legend([names, {'twin estimate (NMPC run)'}]); grid on
subplot(5,1,3); hold on
for m_ = 1:2, plot(tm, R(m_).pl(:,4)/1e3, 'Color', c(m_,:)); end
ylabel('charge accepted [kW]'); grid on
subplot(5,1,4); hold on
for m_ = 1:2, plot(tm, R(m_).pl(:,1)/1e3, 'Color', c(m_,:)); end
ylabel('cooling + heating [kW]'); grid on
subplot(5,1,5); hold on
for m_ = 1:2, plot(tm, R(m_).x(:,4)*100, 'Color', c(m_,:)); end
ylabel('SOC [%]'); xlabel('time [min]'); grid on
