function mdl = build_battery_model(mdl)
%BUILD_BATTERY_MODEL  Create the Simulink model battery_thermal_twin.slx.
%   Top level:  Mission -> Battery Pack and Thermal Loop -> Sensors
%               -> [Rule-based BMS | Digital Twin + NMPC] -> Controller select
%   The pack, the sensors and the rule-based BMS are drawn with basic blocks
%   (Sum, Gain, Product, Fcn, Integrator, Relay, logic) so every heat flow is
%   visible. The twin + NMPC is a MATLAB Function block calling
%   battery_twin_step.m (EKF + fmincon).
%
%   Run setup_battery first so the block parameters resolve.

if nargin < 1, mdl = 'battery_thermal_twin'; end
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'StopTime', 'Tstop', 'Solver', 'ode23', 'MaxStep', '5', ...
    'ReturnWorkspaceOutputs', 'on', 'InitFcn', 'clear battery_twin_step');

build_pack([mdl '/Battery Pack and Thermal Loop']);
build_sensors([mdl '/Sensors']);
build_rule_bms([mdl '/Rule-based BMS']);
build_twin([mdl '/Digital Twin and NMPC']);

% ---------------- top level ----------------
B(mdl, 'simulink/Sources/From Workspace', 'Mission', G(0, 2, 90, 40), ...
    'VariableName', 'dist', 'OutputAfterFinalValue', 'Holding final value');
set_param([mdl '/Battery Pack and Thermal Loop'], 'Position', G(5, 2, 160, 90));
set_param([mdl '/Sensors'],                       'Position', G(7, 2.2, 90, 50));
set_param([mdl '/Rule-based BMS'],                'Position', G(2, 3.6, 140, 70));
set_param([mdl '/Digital Twin and NMPC'],         'Position', G(2, 0.4, 140, 80));
B(mdl, 'simulink/Sources/Constant', 'ctrl_mode (1 BMS, 2 NMPC)', G(2.6, 2.3, 60, 30), 'Value', 'ctrl_mode');
B(mdl, 'simulink/Signal Routing/Switch', 'Controller select', G(4, 1.8, 40, 80), ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '1.5');
B(mdl, 'simulink/Discrete/Unit Delay', 'Last applied u', G(3, 5, 50, 34), ...
    'SampleTime', 'Ts_twin', 'InitialCondition', '[0.3;0;0;0;0]');
B(mdl, 'simulink/Sinks/Scope', 'Pack temperatures', G(8, 0.6, 40, 40));
B(mdl, 'simulink/Sinks/Scope', 'Actuators', G(5, 0.4, 40, 40));
B(mdl, 'simulink/Sinks/To Workspace', 'log u', G(5, 4.6, 70, 30), ...
    'VariableName', 'log_u', 'SaveFormat', 'Array', 'SampleTime', '1');
B(mdl, 'simulink/Sinks/To Workspace', 'log xhat', G(3.4, -0.4, 70, 30), ...
    'VariableName', 'log_xhat', 'SaveFormat', 'Array', 'SampleTime', '1');

L(mdl, 'Mission/1', 'Battery Pack and Thermal Loop/2', 'd = [P_drive, P_chg_max, v, T_amb]');
L(mdl, 'Mission/1', 'Rule-based BMS/2');
L(mdl, 'Battery Pack and Thermal Loop/1', 'Sensors/1', 'x = [T_c, T_s, T_w, SOC]');
L(mdl, 'Battery Pack and Thermal Loop/1', 'Pack temperatures/1');
L(mdl, 'Sensors/1', 'Rule-based BMS/1', 'y = [T_s, T_w, SOC] measured');
L(mdl, 'Sensors/1', 'Digital Twin and NMPC/1');
L(mdl, 'Last applied u/1', 'Digital Twin and NMPC/2');
L(mdl, 'Digital Twin and NMPC/1', 'Controller select/1', 'u_nmpc');
L(mdl, 'Digital Twin and NMPC/2', 'log xhat/1', 'x_hat (twin estimate)');
L(mdl, 'ctrl_mode (1 BMS, 2 NMPC)/1', 'Controller select/2');
L(mdl, 'Rule-based BMS/1', 'Controller select/3', 'u_bms');
L(mdl, 'Controller select/1', 'Battery Pack and Thermal Loop/1', 'u = [pump, chiller, fan, valve, heater]');
L(mdl, 'Controller select/1', 'Last applied u/1');
L(mdl, 'Controller select/1', 'Actuators/1');
L(mdl, 'Controller select/1', 'log u/1');

tidy(mdl);
save_system(mdl);
fprintf('Built %s.slx\n', mdl);
end

% =====================================================================
function build_pack(sys)
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'u', G(0, 2));
B(sys, 'built-in/Inport', 'd', G(0, 7));
B(sys, 'simulink/Signal Routing/Demux', 'Demux u', G(1, 1, 8, 200), 'Outputs', '5');
B(sys, 'simulink/Signal Routing/Demux', 'Demux d', G(1, 6.5, 8, 150), 'Outputs', '4');
L(sys, 'u/1', 'Demux u/1'); L(sys, 'd/1', 'Demux d/1');
% Demux u: 1 pump, 2 chiller, 3 fan, 4 radiator valve, 5 heater
% Demux d: 1 P_drive, 2 P_chg_max, 3 v, 4 T_amb

% ---- states first (so feedback lines can be drawn)
B(sys, 'simulink/Continuous/Integrator', 'T_c', G(14, 0.5, 34, 34), 'InitialCondition', 'x0(1)');
B(sys, 'simulink/Continuous/Integrator', 'T_s', G(14, 2.5, 34, 34), 'InitialCondition', 'x0(2)');
B(sys, 'simulink/Continuous/Integrator', 'T_w', G(14, 4.5, 34, 34), 'InitialCondition', 'x0(3)');
B(sys, 'simulink/Continuous/Integrator', 'SOC', G(14, 7, 34, 34), 'InitialCondition', 'x0(4)');

% ---- thermal-system actuators and their electrical power
B(sys, 'simulink/User-Defined Functions/Fcn', 'Pump power', G(2, 0), 'Expr', 'b_P_pump_max*u(1)^3');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Fan power', G(2, 0.8), 'Expr', 'b_P_fan_max*u(1)^3');
B(sys, 'simulink/Math Operations/Gain', 'Chiller cooling command', G(2, 1.6), 'Gain', 'b_Q_chill_max');
B(sys, 'simulink/Signal Routing/Mux', 'Mux COP', G(2, 2.4, 6, 40), 'Inputs', '2');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Chiller COP', G(3, 2.4, 70, 34), 'Expr', ...
    '1+b_COP_eta*(u(1)-b_dT_evap+273.15)/(sqrt((u(2)+b_dT_cond-u(1)+b_dT_evap)^2+1)+5)');
B(sys, 'simulink/Math Operations/Divide', 'Compressor power', G(4, 1.8, 40, 40));
B(sys, 'simulink/Math Operations/Gain', 'PTC heater', G(2, 3.2), 'Gain', 'b_Q_heat_max');
B(sys, 'simulink/Math Operations/Sum', 'Thermal-system power', G(5, 0.6, 30, 90), 'Inputs', '++++', 'IconShape', 'rectangular');
L(sys, 'Demux u/1', 'Pump power/1', 'u_pump');
L(sys, 'Demux u/3', 'Fan power/1', 'u_fan');
L(sys, 'Demux u/2', 'Chiller cooling command/1', 'u_chill');
L(sys, 'T_w/1', 'Mux COP/1', 'T_w');
L(sys, 'Demux d/4', 'Mux COP/2', 'T_amb');
L(sys, 'Mux COP/1', 'Chiller COP/1');
L(sys, 'Chiller cooling command/1', 'Compressor power/1', 'Q_chill_cmd');
L(sys, 'Chiller COP/1', 'Compressor power/2', 'COP');
L(sys, 'Demux u/5', 'PTC heater/1', 'u_heat');
L(sys, 'Pump power/1', 'Thermal-system power/1', 'P_pump');
L(sys, 'Fan power/1', 'Thermal-system power/2', 'P_fan');
L(sys, 'Compressor power/1', 'Thermal-system power/3', 'P_comp');
L(sys, 'PTC heater/1', 'Thermal-system power/4', 'Q_heat');

% ---- BMS charge acceptance: SOC taper x temperature derate
B(sys, 'simulink/User-Defined Functions/Fcn', 'SOC taper', G(3, 6.4), 'Expr', '(b_soc_full-u(1))/(b_soc_full-b_soc_taper)');
B(sys, 'simulink/Discontinuities/Saturation', 'Taper limits', G(4, 6.4), 'UpperLimit', '1', 'LowerLimit', '0');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Temperature derate', G(3, 5.6, 70, 34), 'Expr', ...
    '(1/(1+exp((u(1)-b_T_derate_hot)/1.5)))*(0.15+0.85/(1+exp(-(u(1)-b_T_derate_cold)/3)))');
B(sys, 'simulink/Math Operations/Product', 'Charge power accepted', G(5, 5.8, 40, 60), 'Inputs', '3');
L(sys, 'SOC/1', 'SOC taper/1', 'SOC');
L(sys, 'SOC taper/1', 'Taper limits/1');
L(sys, 'T_c/1', 'Temperature derate/1', 'T_c');
L(sys, 'Demux d/2', 'Charge power accepted/1', 'P_chg_max');
L(sys, 'Taper limits/1', 'Charge power accepted/2');
L(sys, 'Temperature derate/1', 'Charge power accepted/3');

% ---- electrical: pack power, Joule heat, SOC
B(sys, 'simulink/Math Operations/Sum', 'Pack power', G(6, 3.5, 30, 70), 'Inputs', '++-', 'IconShape', 'rectangular');
B(sys, 'simulink/Signal Routing/Mux', 'Mux joule', G(7, 3, 6, 60), 'Inputs', '3');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Joule heat I2R', G(8, 3.1, 80, 34), 'Expr', ...
    '(u(1)/(b_V0+b_V1*u(2)))^2*b_R_ref*exp(b_B_R*(1/(u(3)+273.15)-1/298.15))*(1+0.6*(u(2)-0.5)^2)');
B(sys, 'simulink/Math Operations/Gain', 'SOC rate', G(12, 7), 'Gain', '-1/b_E_cap');
L(sys, 'Demux d/1', 'Pack power/1', 'P_drive');
L(sys, 'Thermal-system power/1', 'Pack power/2', 'P_thermal');
L(sys, 'Charge power accepted/1', 'Pack power/3', 'P_chg');
L(sys, 'Pack power/1', 'Mux joule/1', 'P_pack');
L(sys, 'SOC/1', 'Mux joule/2');
L(sys, 'T_c/1', 'Mux joule/3');
L(sys, 'Mux joule/1', 'Joule heat I2R/1');
L(sys, 'Pack power/1', 'SOC rate/1');
L(sys, 'SOC rate/1', 'SOC/1', 'SOC_dot');

% ---- heat paths
B(sys, 'simulink/Math Operations/Gain', 'Pump flow', G(6, 0), 'Gain', 'b_mdot_max');
B(sys, 'simulink/Math Operations/Product', 'Radiator valve flow', G(7, 0.2, 40, 34));
B(sys, 'simulink/Signal Routing/Mux', 'Mux plate', G(7, 1.2, 6, 40), 'Inputs', '2');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Cold plate hA', G(8, 1.2, 80, 34), 'Expr', ...
    '(b_k_sw*(u(1)+1e-3)^0.8+b_hA_sw0)*g_plate_factor*(1-g_plate_temp_coeff*(u(2)-25))');
B(sys, 'simulink/Math Operations/Subtract', 'Ts - Tw', G(9, 2, 30, 34));
B(sys, 'simulink/Math Operations/Product', 'Surface to coolant', G(10, 1.6, 34, 34));
B(sys, 'simulink/User-Defined Functions/Fcn', 'Chiller loss in heat', G(8, 4.4, 80, 34), 'Expr', ...
    '1-g_chiller_amb_coeff*sqrt((u(1)-25)^2+0.25)');
B(sys, 'simulink/Math Operations/Product', 'Chiller cooling actual', G(10, 4.2, 34, 34));
B(sys, 'simulink/User-Defined Functions/Fcn', 'E-drive heat leak', G(8, 5.2, 80, 34), 'Expr', ...
    'g_edrive_leak*sqrt(u(1)^2+1e6)');
ma = '(b_rho_a*b_A_front*(b_k_ram*u(3)+b_v_fan_max*u(4))+0.05)';
qrad = sprintf(['(1-exp(-(1/(1/(b_k_air*%s^0.8)+1/(b_k_cool_rad*(u(5)+1e-3)^0.8)))/(%s*b_cp_a)))' ...
                '*%s*b_cp_a*(u(1)-u(2))'], ma, ma, ma);
B(sys, 'simulink/Signal Routing/Mux', 'Mux rad', G(9, 5.8, 6, 90), 'Inputs', '5');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Radiator heat rejection', G(10, 6.1, 80, 34), 'Expr', qrad);
B(sys, 'simulink/Math Operations/Subtract', 'Tc - Ts', G(10, 0.4, 30, 34));
B(sys, 'simulink/Math Operations/Gain', 'Core to surface', G(11, 0.4), 'Gain', 'b_hA_cs');
B(sys, 'simulink/Math Operations/Subtract', 'Ts - Tamb', G(10, 3, 30, 34));
B(sys, 'simulink/Math Operations/Gain', 'Enclosure loss', G(11, 3), 'Gain', 'b_hA_env');
L(sys, 'Demux u/1', 'Pump flow/1');
L(sys, 'Pump flow/1', 'Radiator valve flow/1', 'mdot');
L(sys, 'Demux u/4', 'Radiator valve flow/2', 'u_rv');
L(sys, 'Pump flow/1', 'Mux plate/1');
L(sys, 'T_s/1', 'Mux plate/2', 'T_s');
L(sys, 'Mux plate/1', 'Cold plate hA/1');
L(sys, 'T_s/1', 'Ts - Tw/1');
L(sys, 'T_w/1', 'Ts - Tw/2');
L(sys, 'Cold plate hA/1', 'Surface to coolant/1', 'hA_sw');
L(sys, 'Ts - Tw/1', 'Surface to coolant/2');
L(sys, 'Demux d/4', 'Chiller loss in heat/1');
L(sys, 'Chiller cooling command/1', 'Chiller cooling actual/1');
L(sys, 'Chiller loss in heat/1', 'Chiller cooling actual/2');
L(sys, 'Demux d/1', 'E-drive heat leak/1');
L(sys, 'T_w/1', 'Mux rad/1');
L(sys, 'Demux d/4', 'Mux rad/2');
L(sys, 'Demux d/3', 'Mux rad/3', 'v');
L(sys, 'Demux u/3', 'Mux rad/4');
L(sys, 'Radiator valve flow/1', 'Mux rad/5', 'mdot_rad');
L(sys, 'Mux rad/1', 'Radiator heat rejection/1');
L(sys, 'T_c/1', 'Tc - Ts/1');
L(sys, 'T_s/1', 'Tc - Ts/2');
L(sys, 'Tc - Ts/1', 'Core to surface/1');
L(sys, 'T_s/1', 'Ts - Tamb/1');
L(sys, 'Demux d/4', 'Ts - Tamb/2');
L(sys, 'Ts - Tamb/1', 'Enclosure loss/1');

% ---- heat balances -> 1/C -> integrators
B(sys, 'simulink/Math Operations/Sum', 'Core heat balance', G(12, 0.3, 30, 50), 'Inputs', '+-', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_c', G(13, 0.5), 'Gain', '1/b_C_c');
B(sys, 'simulink/Math Operations/Sum', 'Surface heat balance', G(12, 2.2, 30, 70), 'Inputs', '+--', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_s', G(13, 2.5), 'Gain', '1/b_C_s');
B(sys, 'simulink/Math Operations/Sum', 'Coolant heat balance', G(12, 4, 30, 110), 'Inputs', '+--++', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_w', G(13, 4.5), 'Gain', '1/b_C_w');
L(sys, 'Joule heat I2R/1', 'Core heat balance/1', 'Q_gen');
L(sys, 'Core to surface/1', 'Core heat balance/2', 'Q_cs');
L(sys, 'Core heat balance/1', 'divide by C_c/1');
L(sys, 'divide by C_c/1', 'T_c/1', 'T_c_dot');
L(sys, 'Core to surface/1', 'Surface heat balance/1');
L(sys, 'Surface to coolant/1', 'Surface heat balance/2', 'Q_sw');
L(sys, 'Enclosure loss/1', 'Surface heat balance/3', 'Q_env');
L(sys, 'Surface heat balance/1', 'divide by C_s/1');
L(sys, 'divide by C_s/1', 'T_s/1', 'T_s_dot');
L(sys, 'Surface to coolant/1', 'Coolant heat balance/1');
L(sys, 'Chiller cooling actual/1', 'Coolant heat balance/2', 'Q_chill');
L(sys, 'Radiator heat rejection/1', 'Coolant heat balance/3', 'Q_rad');
L(sys, 'PTC heater/1', 'Coolant heat balance/4');
L(sys, 'E-drive heat leak/1', 'Coolant heat balance/5', 'Q_leak');
L(sys, 'Coolant heat balance/1', 'divide by C_w/1');
L(sys, 'divide by C_w/1', 'T_w/1', 'T_w_dot');

% ---- ageing, outputs, scopes, logs
B(sys, 'simulink/User-Defined Functions/Fcn', 'Ageing rate', G(15, 6, 70, 34), 'Expr', ...
    'exp(b_B_age*(1/298.15-1/(u(1)+273.15)))');
L(sys, 'T_c/1', 'Ageing rate/1');
B(sys, 'simulink/Signal Routing/Mux', 'Mux x', G(16, 1.5, 6, 120), 'Inputs', '4');
B(sys, 'built-in/Outport', 'x', G(17, 2.2));
L(sys, 'T_c/1', 'Mux x/1'); L(sys, 'T_s/1', 'Mux x/2'); L(sys, 'T_w/1', 'Mux x/3'); L(sys, 'SOC/1', 'Mux x/4');
L(sys, 'Mux x/1', 'x/1');
B(sys, 'simulink/Sinks/Scope', 'Scope temperatures', G(15, 2.5, 40, 40), 'NumInputPorts', '3');
L(sys, 'T_c/1', 'Scope temperatures/1');
L(sys, 'T_s/1', 'Scope temperatures/2');
L(sys, 'T_w/1', 'Scope temperatures/3');
B(sys, 'simulink/Sinks/Scope', 'Scope SOC', G(15, 7.2, 34, 34));
L(sys, 'SOC/1', 'Scope SOC/1');
B(sys, 'simulink/Sinks/Scope', 'Scope charge power', G(6, 6.4, 34, 34));
L(sys, 'Charge power accepted/1', 'Scope charge power/1');
B(sys, 'simulink/Sinks/Scope', 'Scope thermal power', G(6, 1.8, 34, 34));
L(sys, 'Thermal-system power/1', 'Scope thermal power/1');
B(sys, 'simulink/Signal Routing/Mux', 'Mux log', G(16, 4.5, 6, 110), 'Inputs', '6');
B(sys, 'simulink/Sinks/To Workspace', 'log plant', G(17, 5, 70, 30), ...
    'VariableName', 'log_plant', 'SaveFormat', 'Array', 'SampleTime', '1');
L(sys, 'Thermal-system power/1', 'Mux log/1');
L(sys, 'Compressor power/1', 'Mux log/2');
L(sys, 'PTC heater/1', 'Mux log/3');
L(sys, 'Charge power accepted/1', 'Mux log/4');
L(sys, 'Ageing rate/1', 'Mux log/5', 'age_rate');
L(sys, 'Joule heat I2R/1', 'Mux log/6');
L(sys, 'Mux log/1', 'log plant/1');
B(sys, 'simulink/Sinks/To Workspace', 'log x', G(17, 3.2, 70, 30), ...
    'VariableName', 'log_x', 'SaveFormat', 'Array', 'SampleTime', '1');
L(sys, 'Mux x/1', 'log x/1');
tidy(sys);
end

% =====================================================================
function build_sensors(sys)
% The BMS sees cell surface and coolant temperature and SOC, with noise,
% every Ts_twin seconds. The cell-core temperature is NOT measured.
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'x', G(0, 1));
B(sys, 'simulink/Signal Routing/Demux', 'Demux x', G(1, 0.5, 6, 120), 'Outputs', '4');
L(sys, 'x/1', 'Demux x/1');
B(sys, 'simulink/Sinks/Terminator', 'Core not measured', G(2, 0, 20, 20));
L(sys, 'Demux x/1', 'Core not measured/1');
B(sys, 'simulink/Signal Routing/Mux', 'Mux y', G(2, 1, 6, 90), 'Inputs', '3');
L(sys, 'Demux x/2', 'Mux y/1', 'T_s');
L(sys, 'Demux x/3', 'Mux y/2', 'T_w');
L(sys, 'Demux x/4', 'Mux y/3', 'SOC');
B(sys, 'simulink/Sources/Random Number', 'Sensor noise', G(2, 2.6, 50, 34), ...
    'Mean', '[0;0;0]', 'Variance', 'noise_std.^2', 'Seed', '[11;12;13]', 'SampleTime', 'Ts_twin');
B(sys, 'simulink/Math Operations/Sum', 'Add noise', G(3, 1.5, 30, 40), 'Inputs', '++');
B(sys, 'simulink/Discrete/Zero-Order Hold', 'Sample every Ts', G(4, 1.5), 'SampleTime', 'Ts_twin');
B(sys, 'built-in/Outport', 'y', G(5, 1.5));
L(sys, 'Mux y/1', 'Add noise/1');
L(sys, 'Sensor noise/1', 'Add noise/2');
L(sys, 'Add noise/1', 'Sample every Ts/1');
L(sys, 'Sample every Ts/1', 'y/1', 'y measured');
tidy(sys);
end

% =====================================================================
function build_rule_bms(sys)
% Typical production logic: thresholds on measured temperatures, no preview.
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'y', G(0, 1));
B(sys, 'built-in/Inport', 'd', G(0, 4));
B(sys, 'simulink/Signal Routing/Demux', 'Demux y', G(1, 0.6, 6, 90), 'Outputs', '3');
B(sys, 'simulink/Signal Routing/Demux', 'Demux d', G(1, 3.6, 6, 110), 'Outputs', '4');
L(sys, 'y/1', 'Demux y/1'); L(sys, 'd/1', 'Demux d/1');
B(sys, 'simulink/Sinks/Terminator', 'SOC unused', G(2, 1.8, 20, 20));
L(sys, 'Demux y/3', 'SOC unused/1');
B(sys, 'simulink/Sinks/Terminator', 'v unused', G(2, 5, 20, 20));
L(sys, 'Demux d/3', 'v unused/1');

% chiller: on above chill_on, off below chill_off
B(sys, 'simulink/Discontinuities/Relay', 'Chiller relay', G(3, 0), ...
    'OnSwitchValue', 'b_chill_on', 'OffSwitchValue', 'b_chill_off', 'OnOutputValue', '1', 'OffOutputValue', '0');
% heater: heats below heat_on, stops above heat_off (relay with inverted outputs)
B(sys, 'simulink/Discontinuities/Relay', 'Heater relay', G(3, 1), ...
    'OnSwitchValue', 'b_heat_off', 'OffSwitchValue', 'b_heat_on', 'OnOutputValue', '0', 'OffOutputValue', '1');
L(sys, 'Demux y/1', 'Chiller relay/1', 'T_s');
L(sys, 'Demux y/1', 'Heater relay/1');

% pump: 0.6 when the loop is busy (driving hard, charging, chilling, heating), else 0.3
B(sys, 'simulink/Math Operations/Abs', 'abs P_drive', G(2, 3.5, 34, 34));
B(sys, 'simulink/Logic and Bit Operations/Compare To Constant', 'Driving hard', G(3, 3.5), 'relop', '>', 'const', '20e3');
B(sys, 'simulink/Logic and Bit Operations/Compare To Constant', 'Charging', G(3, 4.3), 'relop', '>', 'const', '0');
B(sys, 'simulink/Logic and Bit Operations/Compare To Constant', 'Chiller on', G(4, 0), 'relop', '>', 'const', '0.5');
B(sys, 'simulink/Logic and Bit Operations/Compare To Constant', 'Heater on', G(4, 1), 'relop', '>', 'const', '0.5');
B(sys, 'simulink/Logic and Bit Operations/Logical Operator', 'Loop busy', G(5, 2.5, 40, 80), 'Operator', 'OR', 'Inputs', '4');
B(sys, 'simulink/Sources/Constant', 'Pump high', G(5, 1.6, 40, 26), 'Value', '0.6');
B(sys, 'simulink/Sources/Constant', 'Pump low', G(5, 3.8, 40, 26), 'Value', '0.3');
B(sys, 'simulink/Signal Routing/Switch', 'Pump speed', G(6, 2.5, 40, 70), 'Criteria', 'u2 ~= 0');
L(sys, 'Demux d/1', 'abs P_drive/1', 'P_drive');
L(sys, 'abs P_drive/1', 'Driving hard/1');
L(sys, 'Demux d/2', 'Charging/1', 'P_chg_max');
L(sys, 'Chiller relay/1', 'Chiller on/1');
L(sys, 'Heater relay/1', 'Heater on/1');
L(sys, 'Driving hard/1', 'Loop busy/1');
L(sys, 'Charging/1', 'Loop busy/2');
L(sys, 'Chiller on/1', 'Loop busy/3');
L(sys, 'Heater on/1', 'Loop busy/4');
L(sys, 'Pump high/1', 'Pump speed/1');
L(sys, 'Loop busy/1', 'Pump speed/2');
L(sys, 'Pump low/1', 'Pump speed/3');

% radiator: open when not chilling, surface > 28 degC and coolant > ambient + 3 K
B(sys, 'simulink/Logic and Bit Operations/Logical Operator', 'Not chilling', G(5, 0, 40, 34), 'Operator', 'NOT', 'Inputs', '1');
B(sys, 'simulink/Logic and Bit Operations/Compare To Constant', 'Surface above 28', G(3, 2), 'relop', '>', 'const', '28');
B(sys, 'simulink/Math Operations/Bias', 'T_amb + 3', G(2, 6), 'Bias', '3');
B(sys, 'simulink/Logic and Bit Operations/Relational Operator', 'Coolant warmer than air', G(3, 5.5, 40, 40), 'Operator', '>');
B(sys, 'simulink/Logic and Bit Operations/Logical Operator', 'Radiate', G(6, 0.6, 40, 70), 'Operator', 'AND', 'Inputs', '3');
B(sys, 'simulink/Signal Attributes/Data Type Conversion', 'Valve open', G(7, 0.8), 'OutDataTypeStr', 'double');
L(sys, 'Chiller on/1', 'Not chilling/1');
L(sys, 'Demux y/1', 'Surface above 28/1');
L(sys, 'Demux d/4', 'T_amb + 3/1', 'T_amb');
L(sys, 'Demux y/2', 'Coolant warmer than air/1', 'T_w');
L(sys, 'T_amb + 3/1', 'Coolant warmer than air/2');
L(sys, 'Not chilling/1', 'Radiate/1');
L(sys, 'Surface above 28/1', 'Radiate/2');
L(sys, 'Coolant warmer than air/1', 'Radiate/3');
L(sys, 'Radiate/1', 'Valve open/1');

% fan: proportional to (T_w - T_amb - 3)/10, only while radiating
B(sys, 'simulink/Math Operations/Subtract', 'Tw - Tamb', G(4, 6.4, 30, 34));
B(sys, 'simulink/Math Operations/Bias', 'minus 3 K', G(5, 6.4), 'Bias', '-3');
B(sys, 'simulink/Math Operations/Gain', 'Fan gain', G(6, 6.4), 'Gain', '0.1');
B(sys, 'simulink/Discontinuities/Saturation', 'Fan limits', G(7, 6.4), 'UpperLimit', '1', 'LowerLimit', '0');
B(sys, 'simulink/Math Operations/Product', 'Fan only when radiating', G(8, 5, 40, 40));
L(sys, 'Demux y/2', 'Tw - Tamb/1');
L(sys, 'Demux d/4', 'Tw - Tamb/2');
L(sys, 'Tw - Tamb/1', 'minus 3 K/1');
L(sys, 'minus 3 K/1', 'Fan gain/1');
L(sys, 'Fan gain/1', 'Fan limits/1');
L(sys, 'Valve open/1', 'Fan only when radiating/1');
L(sys, 'Fan limits/1', 'Fan only when radiating/2');

B(sys, 'simulink/Signal Routing/Mux', 'Mux u', G(9, 1.5, 6, 160), 'Inputs', '5');
B(sys, 'built-in/Outport', 'u', G(10, 2.4));
L(sys, 'Pump speed/1', 'Mux u/1', 'u_pump');
L(sys, 'Chiller relay/1', 'Mux u/2', 'u_chill');
L(sys, 'Fan only when radiating/1', 'Mux u/3', 'u_fan');
L(sys, 'Valve open/1', 'Mux u/4', 'u_rv');
L(sys, 'Heater relay/1', 'Mux u/5', 'u_heat');
L(sys, 'Mux u/1', 'u/1');
tidy(sys);
end

% =====================================================================
function build_twin(sys)
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'y', G(0, 0));
B(sys, 'built-in/Inport', 'u_applied', G(0, 1));
B(sys, 'simulink/Sources/Clock', 'Clock', G(0, 2));
B(sys, 'simulink/Sources/Constant', 'mode', G(1, 2.8, 40, 26), 'Value', 'ctrl_mode');
B(sys, 'simulink/Discrete/Zero-Order Hold', 'Sample t', G(1, 2), 'SampleTime', 'Ts_twin');
blk = [sys '/EKF twin and NMPC'];
B(sys, 'simulink/User-Defined Functions/MATLAB Function', 'EKF twin and NMPC', G(2.3, 0.3, 140, 120));
set_mfunction(blk, sprintf([ ...
    'function [u, xhat] = twin(y, u_applied, t, mode)\n' ...
    '%% EKF virtual sensor + NMPC in battery_twin_step.m, every Ts_twin seconds.\n' ...
    'coder.extrinsic(''battery_twin_step'');\n' ...
    'u = zeros(5,1);\n' ...
    'xhat = zeros(4,1);\n' ...
    '[u, xhat] = battery_twin_step(y, u_applied, t, mode);\n']));
B(sys, 'built-in/Outport', 'u', G(4, 0.4));
B(sys, 'built-in/Outport', 'x_hat', G(4, 1.4));
L(sys, 'y/1', 'EKF twin and NMPC/1', 'y');
L(sys, 'u_applied/1', 'EKF twin and NMPC/2', 'u_applied');
L(sys, 'Clock/1', 'Sample t/1');
L(sys, 'Sample t/1', 'EKF twin and NMPC/3', 't');
L(sys, 'mode/1', 'EKF twin and NMPC/4');
L(sys, 'EKF twin and NMPC/1', 'u/1', 'u');
L(sys, 'EKF twin and NMPC/2', 'x_hat/1', 'x_hat');
tidy(sys);
end

% =====================================================================
function set_mfunction(blkPath, code)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blkPath);
ch.Script = code;
end

function h = B(sys, lib, name, pos, varargin)
h = add_block(lib, [sys '/' name], 'Position', pos, varargin{:});
end

function L(sys, src, dst, label)
h = add_line(sys, src, dst, 'autorouting', 'on');
if nargin > 3 && ~isempty(label)
    set_param(h, 'Name', label);
end
end

function p = G(col, row, w, h)
if nargin < 3, w = 60; end
if nargin < 4, h = 34; end
x = 30 + col*130;
y = 30 + max(row, -0.3)*80 + 30;
p = round([x, y, x + w, y + h]);
end

function tidy(sys)
try
    Simulink.BlockDiagram.arrangeSystem(sys);
catch
end
end
