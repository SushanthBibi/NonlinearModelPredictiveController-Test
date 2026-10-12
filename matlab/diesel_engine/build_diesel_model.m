function mdl = build_diesel_model(mdl)
%BUILD_DIESEL_MODEL  Create the Simulink model diesel_cooling_nmpc.slx.
%   Top level:  Drive cycle -> [Conventional | NMPC] -> Controller select
%               -> Engine and Cooling Plant -> scopes / logs
%   The plant and the conventional controller are drawn with basic blocks
%   (Sum, Gain, Product, Fcn, Integrator, Relay, Saturation, Scope) so every
%   heat flow can be inspected. The NMPC is a MATLAB Function block that
%   calls diesel_nmpc_step.m (fmincon).
%
%   Run setup_diesel first so the block parameters resolve.

if nargin < 1, mdl = 'diesel_cooling_nmpc'; end
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);
set_param(mdl, 'StopTime', 'Tstop', 'Solver', 'ode23', 'MaxStep', '1', ...
    'ReturnWorkspaceOutputs', 'on', 'InitFcn', 'clear diesel_nmpc_step');

build_plant([mdl '/Engine and Cooling Plant']);
build_conventional([mdl '/Conventional Controller']);
build_nmpc([mdl '/NMPC Controller']);

% ---------------- top level ----------------
B(mdl, 'simulink/Sources/From Workspace', 'Drive cycle', G(0, 2, 90, 40), ...
    'VariableName', 'dist', 'OutputAfterFinalValue', 'Holding final value');
set_param([mdl '/Engine and Cooling Plant'], 'Position', G(4, 2, 150, 90));
set_param([mdl '/Conventional Controller'],  'Position', G(2, 3.5, 140, 70));
set_param([mdl '/NMPC Controller'],          'Position', G(2, 0.5, 140, 70));
B(mdl, 'simulink/Sources/Constant', 'ctrl_mode (1 conv, 2 NMPC)', G(2, 2.2, 60, 30), 'Value', 'ctrl_mode');
B(mdl, 'simulink/Signal Routing/Switch', 'Controller select', G(3, 1.8, 40, 70), ...
    'Criteria', 'u2 >= Threshold', 'Threshold', '1.5');
B(mdl, 'simulink/Sinks/Scope', 'Temperatures T_e T_r T_o', G(6, 1, 40, 40));
B(mdl, 'simulink/Sinks/Scope', 'Actuators pump fan thermostat', G(4, 0.3, 40, 40));
B(mdl, 'simulink/Sinks/To Workspace', 'log u', G(4, 4.4, 70, 30), ...
    'VariableName', 'log_u', 'SaveFormat', 'Array', 'SampleTime', '1');

L(mdl, 'Drive cycle/1', 'Engine and Cooling Plant/2', 'd = [N, P_b, v, T_amb]');
L(mdl, 'Drive cycle/1', 'Conventional Controller/2');
L(mdl, 'Drive cycle/1', 'NMPC Controller/2');
L(mdl, 'NMPC Controller/1', 'Controller select/1', 'u_nmpc');
L(mdl, 'ctrl_mode (1 conv, 2 NMPC)/1', 'Controller select/2');
L(mdl, 'Conventional Controller/1', 'Controller select/3', 'u_conv');
L(mdl, 'Controller select/1', 'Engine and Cooling Plant/1', 'u = [u_pump, u_fan, u_th]');
L(mdl, 'Controller select/1', 'Actuators pump fan thermostat/1');
L(mdl, 'Controller select/1', 'log u/1');
L(mdl, 'Engine and Cooling Plant/1', 'Temperatures T_e T_r T_o/1', 'x = [T_e, T_r, T_o]');
L(mdl, 'Engine and Cooling Plant/1', 'Conventional Controller/1');
L(mdl, 'Engine and Cooling Plant/1', 'NMPC Controller/1');

tidy(mdl);
save_system(mdl);
fprintf('Built %s.slx\n', mdl);
end

% =====================================================================
function build_plant(sys)
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'u', G(0, 2));
B(sys, 'built-in/Inport', 'd', G(0, 7));
B(sys, 'simulink/Signal Routing/Demux', 'Demux u', G(1, 1.5, 8, 120), 'Outputs', '3');
B(sys, 'simulink/Signal Routing/Demux', 'Demux d', G(1, 6.5, 8, 140), 'Outputs', '4');
L(sys, 'u/1', 'Demux u/1'); L(sys, 'd/1', 'Demux d/1');
% Demux u: 1 u_pump, 2 u_fan, 3 u_th ;  Demux d: 1 N, 2 P_b, 3 v, 4 T_amb

% --- actuators
B(sys, 'simulink/User-Defined Functions/Fcn', 'Pump power', G(2, 0), 'Expr', 'p_P_pump_max*u(1)^3');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Fan power',  G(2, 1), 'Expr', 'p_P_fan_max*u(1)^3');
B(sys, 'simulink/Math Operations/Gain', 'Pump flow', G(2, 2), 'Gain', 'p_mdot_max');
B(sys, 'simulink/Math Operations/Product', 'Radiator flow', G(3, 2.3));
B(sys, 'simulink/User-Defined Functions/Fcn', 'Oil cooler hA', G(2, 3), 'Expr', 'p_hA_oc0+p_hA_oc1*u(1)');
L(sys, 'Demux u/1', 'Pump power/1', 'u_pump');
L(sys, 'Demux u/2', 'Fan power/1', 'u_fan');
L(sys, 'Demux u/1', 'Pump flow/1');
L(sys, 'Demux u/1', 'Oil cooler hA/1');
L(sys, 'Pump flow/1', 'Radiator flow/1', 'mdot');
L(sys, 'Demux u/3', 'Radiator flow/2', 'u_th');

% --- friction and fuel
B(sys, 'simulink/Signal Routing/Mux', 'Mux fric', G(3, 5, 6, 40), 'Inputs', '2');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Friction power', G(4, 5, 70, 34), 'Expr', ...
    'p_P_fric_ref*(u(1)/1000)^1.5*(0.5+0.5*exp(p_visc_B*(1/(u(2)+273.15)-1/(p_T_oil_ref+273.15))))');
B(sys, 'simulink/Math Operations/Sum', 'Crank power', G(5, 4, 30, 80), 'Inputs', '++++', 'IconShape', 'rectangular');
B(sys, 'simulink/User-Defined Functions/Fcn', 'No fuel when braking', G(6, 4.3), 'Expr', '0.5*(u(1)+sqrt(u(1)^2+1e6))');
B(sys, 'simulink/Math Operations/Gain', 'Fuel power', G(7, 4.3), 'Gain', '1/p_eta_ind');
B(sys, 'simulink/Math Operations/Gain', 'Fuel mass flow', G(8, 5.2), 'Gain', '1/p_LHV');
L(sys, 'Demux d/1', 'Mux fric/1', 'N');
L(sys, 'Mux fric/1', 'Friction power/1');
L(sys, 'Demux d/2', 'Crank power/1', 'P_b');
L(sys, 'Friction power/1', 'Crank power/2', 'P_fric');
L(sys, 'Pump power/1', 'Crank power/3', 'P_pump');
L(sys, 'Fan power/1', 'Crank power/4', 'P_fan');
L(sys, 'Crank power/1', 'No fuel when braking/1');
L(sys, 'No fuel when braking/1', 'Fuel power/1');
L(sys, 'Fuel power/1', 'Fuel mass flow/1', 'P_fuel');

% --- heat release split
B(sys, 'simulink/Math Operations/Gain', 'Coolant share of fuel',     G(8, 3.5), 'Gain', 'p_f_cool_fuel');
B(sys, 'simulink/Math Operations/Gain', 'Coolant share of friction', G(8, 2.7), 'Gain', 'p_f_cool_fric');
B(sys, 'simulink/Math Operations/Sum', 'Q_cool', G(9, 3, 30, 40), 'Inputs', '++');
B(sys, 'simulink/Math Operations/Gain', 'Oil share of fuel',     G(8, 6.2), 'Gain', 'p_f_oil_fuel');
B(sys, 'simulink/Math Operations/Gain', 'Oil share of friction', G(8, 7), 'Gain', '1-p_f_cool_fric');
B(sys, 'simulink/Math Operations/Sum', 'Q_oil', G(9, 6.5, 30, 40), 'Inputs', '++');
L(sys, 'Fuel power/1', 'Coolant share of fuel/1');
L(sys, 'Friction power/1', 'Coolant share of friction/1');
L(sys, 'Coolant share of friction/1', 'Q_cool/1');
L(sys, 'Coolant share of fuel/1', 'Q_cool/2');
L(sys, 'Fuel power/1', 'Oil share of fuel/1');
L(sys, 'Friction power/1', 'Oil share of friction/1');
L(sys, 'Oil share of fuel/1', 'Q_oil/1');
L(sys, 'Oil share of friction/1', 'Q_oil/2');

% --- radiator heat rejection: effectiveness on the air side
ma = '(p_rho_a*p_A_front*(p_k_ram*u(3)+p_v_fan_max*u(4))+0.05)';
qrad = sprintf(['(1-exp(-(1/(1/(p_k_air*%s^0.8)+1/(p_k_cool*(u(5)+1e-3)^0.8)))/(%s*p_cp_a)))' ...
                '*%s*p_cp_a*(u(1)-u(2))'], ma, ma, ma);
B(sys, 'simulink/Signal Routing/Mux', 'Mux rad', G(10, 0, 6, 90), 'Inputs', '5');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Radiator heat rejection', G(11, 0.3, 80, 34), 'Expr', qrad);

% --- couplings
B(sys, 'simulink/Math Operations/Subtract', 'Te - Tr', G(10, 1.8, 30, 34));
B(sys, 'simulink/Math Operations/Product', 'Advection', G(11, 1.8, 30, 34));
B(sys, 'simulink/Math Operations/Gain', 'Coolant cp', G(12, 1.8), 'Gain', 'p_cp_c');
B(sys, 'simulink/Math Operations/Subtract', 'To - Te', G(10, 4.5, 30, 34));
B(sys, 'simulink/Math Operations/Product', 'Oil to coolant', G(11, 4.5, 30, 34));
B(sys, 'simulink/Math Operations/Subtract', 'Te - Tamb', G(10, 3.6, 30, 34));
B(sys, 'simulink/Math Operations/Gain', 'Engine losses', G(11, 3.6), 'Gain', 'p_hA_amb');

% --- heat balances -> 1/C -> integrators (states)
B(sys, 'simulink/Math Operations/Sum', 'Engine heat balance', G(13, 3, 30, 90), 'Inputs', '++--', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_e', G(14, 3.3), 'Gain', '1/p_C_e');
B(sys, 'simulink/Continuous/Integrator', 'T_e', G(15, 3.3, 34, 34), 'InitialCondition', 'x0(1)');
B(sys, 'simulink/Math Operations/Sum', 'Radiator heat balance', G(13, 1, 30, 50), 'Inputs', '+-', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_r', G(14, 1.1), 'Gain', '1/p_C_r');
B(sys, 'simulink/Continuous/Integrator', 'T_r', G(15, 1.1, 34, 34), 'InitialCondition', 'x0(2)');
B(sys, 'simulink/Math Operations/Sum', 'Oil heat balance', G(13, 6, 30, 50), 'Inputs', '+-', 'IconShape', 'rectangular');
B(sys, 'simulink/Math Operations/Gain', 'divide by C_o', G(14, 6.1), 'Gain', '1/p_C_o');
B(sys, 'simulink/Continuous/Integrator', 'T_o', G(15, 6.1, 34, 34), 'InitialCondition', 'x0(3)');

% radiator inputs
L(sys, 'T_r/1', 'Mux rad/1', 'T_r');
L(sys, 'Demux d/4', 'Mux rad/2', 'T_amb');
L(sys, 'Demux d/3', 'Mux rad/3', 'v');
L(sys, 'Demux u/2', 'Mux rad/4');
L(sys, 'Radiator flow/1', 'Mux rad/5', 'mdot_rad');
L(sys, 'Mux rad/1', 'Radiator heat rejection/1');
% advection to radiator
L(sys, 'T_e/1', 'Te - Tr/1', 'T_e');
L(sys, 'T_r/1', 'Te - Tr/2');
L(sys, 'Radiator flow/1', 'Advection/1');
L(sys, 'Te - Tr/1', 'Advection/2');
L(sys, 'Advection/1', 'Coolant cp/1');
% oil cooler
L(sys, 'T_o/1', 'To - Te/1', 'T_o');
L(sys, 'T_e/1', 'To - Te/2');
L(sys, 'Oil cooler hA/1', 'Oil to coolant/1', 'hA_oc');
L(sys, 'To - Te/1', 'Oil to coolant/2');
% block losses
L(sys, 'T_e/1', 'Te - Tamb/1');
L(sys, 'Demux d/4', 'Te - Tamb/2');
L(sys, 'Te - Tamb/1', 'Engine losses/1');
% balances
L(sys, 'Q_cool/1', 'Engine heat balance/1', 'Q_cool');
L(sys, 'Oil to coolant/1', 'Engine heat balance/2', 'Q_oc');
L(sys, 'Coolant cp/1', 'Engine heat balance/3', 'Q_adv');
L(sys, 'Engine losses/1', 'Engine heat balance/4', 'Q_loss');
L(sys, 'Engine heat balance/1', 'divide by C_e/1');
L(sys, 'divide by C_e/1', 'T_e/1', 'T_e_dot');
L(sys, 'Coolant cp/1', 'Radiator heat balance/1');
L(sys, 'Radiator heat rejection/1', 'Radiator heat balance/2', 'Q_rad');
L(sys, 'Radiator heat balance/1', 'divide by C_r/1');
L(sys, 'divide by C_r/1', 'T_r/1', 'T_r_dot');
L(sys, 'Q_oil/1', 'Oil heat balance/1', 'Q_oil');
L(sys, 'Oil to coolant/1', 'Oil heat balance/2');
L(sys, 'Oil heat balance/1', 'divide by C_o/1');
L(sys, 'divide by C_o/1', 'T_o/1', 'T_o_dot');
L(sys, 'T_o/1', 'Mux fric/2');

% --- liner metal temperature
B(sys, 'simulink/Signal Routing/Mux', 'Mux metal', G(16, 4.5, 6, 60), 'Inputs', '3');
B(sys, 'simulink/User-Defined Functions/Fcn', 'Liner metal temp', G(17, 4.7, 70, 34), ...
    'Expr', 'u(1)+u(2)/(p_k_wall*(u(3)+1e-3)^0.8)');
L(sys, 'T_e/1', 'Mux metal/1');
L(sys, 'Q_cool/1', 'Mux metal/2');
L(sys, 'Pump flow/1', 'Mux metal/3');
L(sys, 'Mux metal/1', 'Liner metal temp/1');

% --- outputs, scopes, logs
B(sys, 'simulink/Signal Routing/Mux', 'Mux x', G(17, 2, 6, 80), 'Inputs', '3');
B(sys, 'built-in/Outport', 'x', G(18, 2.3));
L(sys, 'T_e/1', 'Mux x/1'); L(sys, 'T_r/1', 'Mux x/2'); L(sys, 'T_o/1', 'Mux x/3');
L(sys, 'Mux x/1', 'x/1');
B(sys, 'simulink/Sinks/Scope', 'Scope T_e', G(16, 3.2, 34, 34));
L(sys, 'T_e/1', 'Scope T_e/1');
B(sys, 'simulink/Sinks/Scope', 'Scope T_r', G(16, 0.6, 34, 34));
L(sys, 'T_r/1', 'Scope T_r/1');
B(sys, 'simulink/Sinks/Scope', 'Scope T_o', G(16, 6.6, 34, 34));
L(sys, 'T_o/1', 'Scope T_o/1');
B(sys, 'simulink/Sinks/Scope', 'Scope liner metal', G(18, 4.7, 34, 34));
L(sys, 'Liner metal temp/1', 'Scope liner metal/1', 'T_metal');
B(sys, 'simulink/Sinks/Scope', 'Scope fan and pump power', G(4, 0.5, 34, 34), 'NumInputPorts', '2');
L(sys, 'Fan power/1', 'Scope fan and pump power/1');
L(sys, 'Pump power/1', 'Scope fan and pump power/2');
B(sys, 'simulink/Signal Routing/Mux', 'Mux log', G(18, 6.5, 6, 90), 'Inputs', '5');
B(sys, 'simulink/Sinks/To Workspace', 'log plant', G(19, 6.8, 70, 30), ...
    'VariableName', 'log_plant', 'SaveFormat', 'Array', 'SampleTime', '1');
L(sys, 'Fan power/1', 'Mux log/1');
L(sys, 'Pump power/1', 'Mux log/2');
L(sys, 'Friction power/1', 'Mux log/3');
L(sys, 'Fuel mass flow/1', 'Mux log/4', 'mdot_fuel');
L(sys, 'Liner metal temp/1', 'Mux log/5');
L(sys, 'Mux log/1', 'log plant/1');
B(sys, 'simulink/Sinks/To Workspace', 'log x', G(18, 3.4, 70, 30), ...
    'VariableName', 'log_x', 'SaveFormat', 'Array', 'SampleTime', '1');
L(sys, 'Mux x/1', 'log x/1');
tidy(sys);
end

% =====================================================================
function build_conventional(sys)
% Production cooling system: wax thermostat, crank-driven pump, on/off viscous fan.
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'x', G(0, 1));
B(sys, 'built-in/Inport', 'd', G(0, 3));
B(sys, 'simulink/Signal Routing/Demux', 'Demux x', G(1, 0.6, 6, 80), 'Outputs', '3');
B(sys, 'simulink/Signal Routing/Demux', 'Demux d', G(1, 2.6, 6, 90), 'Outputs', '4');
L(sys, 'x/1', 'Demux x/1'); L(sys, 'd/1', 'Demux d/1');
for k = 2:3, tb = sprintf('Term x%d', k); B(sys, 'simulink/Sinks/Terminator', tb, G(2, 1+0.4*k, 20, 20)); L(sys, sprintf('Demux x/%d', k), [tb '/1']); end
for k = 2:4, tb = sprintf('Term d%d', k); B(sys, 'simulink/Sinks/Terminator', tb, G(2, 2.8+0.4*k, 20, 20)); L(sys, sprintf('Demux d/%d', k), [tb '/1']); end

% thermostat: opening = sat((T_e - T_open)/(T_full - T_open), 0, 1)
B(sys, 'simulink/Math Operations/Bias', 'minus T_open', G(3, 0), 'Bias', '-p_T_open');
B(sys, 'simulink/Math Operations/Gain', 'Thermostat gain', G(4, 0), 'Gain', '1/(p_T_full-p_T_open)');
B(sys, 'simulink/Discontinuities/Saturation', 'Wax thermostat', G(5, 0), 'UpperLimit', '1', 'LowerLimit', '0');
% fan: on at fan_on, off at fan_off (hysteresis)
B(sys, 'simulink/Discontinuities/Relay', 'Viscous fan', G(4, 1), ...
    'OnSwitchValue', 'p_fan_on', 'OffSwitchValue', 'p_fan_off', 'OnOutputValue', '1', 'OffOutputValue', '0');
% pump: speed proportional to engine speed
B(sys, 'simulink/Math Operations/Gain', 'Crank-driven pump', G(3, 2.6), 'Gain', '1/p_N_pump_full');
B(sys, 'simulink/Discontinuities/Saturation', 'Pump speed limits', G(4, 2.6), 'UpperLimit', '1', 'LowerLimit', 'p_u_pump_min');
B(sys, 'simulink/Signal Routing/Mux', 'Mux u', G(6, 0.6, 6, 120), 'Inputs', '3');
B(sys, 'built-in/Outport', 'u', G(7, 1.2));
L(sys, 'Demux x/1', 'minus T_open/1', 'T_e');
L(sys, 'minus T_open/1', 'Thermostat gain/1');
L(sys, 'Thermostat gain/1', 'Wax thermostat/1');
L(sys, 'Demux x/1', 'Viscous fan/1');
L(sys, 'Demux d/1', 'Crank-driven pump/1', 'N');
L(sys, 'Crank-driven pump/1', 'Pump speed limits/1');
L(sys, 'Pump speed limits/1', 'Mux u/1', 'u_pump');
L(sys, 'Viscous fan/1', 'Mux u/2', 'u_fan');
L(sys, 'Wax thermostat/1', 'Mux u/3', 'u_th');
L(sys, 'Mux u/1', 'u/1');
tidy(sys);
end

% =====================================================================
function build_nmpc(sys)
add_block('built-in/Subsystem', sys);
B(sys, 'built-in/Inport', 'x', G(0, 0));
B(sys, 'built-in/Inport', 'd', G(0, 2));
B(sys, 'simulink/Sinks/Terminator', 'Preview read from route plan', G(1, 2, 20, 20));
L(sys, 'd/1', 'Preview read from route plan/1');
B(sys, 'simulink/Sources/Clock', 'Clock', G(0, 1));
B(sys, 'simulink/Sources/Constant', 'mode', G(1, 1.6, 40, 26), 'Value', 'ctrl_mode');
B(sys, 'simulink/Discrete/Zero-Order Hold', 'Sample x', G(1, 0), 'SampleTime', 'Ts_mpc');
B(sys, 'simulink/Discrete/Zero-Order Hold', 'Sample t', G(1, 1), 'SampleTime', 'Ts_mpc');
blk = [sys '/NMPC fmincon'];
B(sys, 'simulink/User-Defined Functions/MATLAB Function', 'NMPC fmincon', G(2.3, 0.3, 120, 90));
set_mfunction(blk, sprintf([ ...
    'function u = nmpc(x, t, mode)\n' ...
    '%% Calls the optimiser in diesel_nmpc_step.m every Ts_mpc seconds.\n' ...
    'coder.extrinsic(''diesel_nmpc_step'');\n' ...
    'u = zeros(3,1);\n' ...
    'u = diesel_nmpc_step(x, t, mode);\n']));
B(sys, 'built-in/Outport', 'u', G(4, 0.6));
L(sys, 'x/1', 'Sample x/1');
L(sys, 'Clock/1', 'Sample t/1');
L(sys, 'Sample x/1', 'NMPC fmincon/1', 'x');
L(sys, 'Sample t/1', 'NMPC fmincon/2', 't');
L(sys, 'mode/1', 'NMPC fmincon/3');
L(sys, 'NMPC fmincon/1', 'u/1', 'u');
tidy(sys);
end

% =====================================================================
function set_mfunction(blkPath, code)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', blkPath);
ch.Script = code;
end

function h = B(sys, lib, name, pos, varargin)
if nargin < 4 || isempty(pos), pos = G(0, 0); end
h = add_block(lib, [sys '/' name], 'Position', pos, varargin{:});
end

function L(sys, src, dst, label)
h = add_line(sys, src, dst, 'autorouting', 'on');
if nargin > 3 && ~isempty(label)
    set_param(h, 'Name', label);
end
end

function p = G(col, row, w, h)
% grid position helper: column/row -> [left top right bottom]
if nargin < 3, w = 60; end
if nargin < 4, h = 34; end
x = 30 + col*130;
y = 30 + row*80;
p = round([x, y, x + w, y + h]);
end

function tidy(sys)
% Auto-arrange (R2019b+); keep the hand layout on older releases.
try
    Simulink.BlockDiagram.arrangeSystem(sys);
catch
end
end
