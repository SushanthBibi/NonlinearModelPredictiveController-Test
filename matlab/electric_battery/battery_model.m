function [xdot, y] = battery_model(x, u, d, P, G)
%BATTERY_MODEL  Electro-thermal model of the pack and its thermal loop.
%   x = [T_c; T_s; T_w; SOC]                 cell core, cell surface, coolant [degC], SOC [-]
%   u = [u_pump; u_chill; u_fan; u_rv; u_heat]  all in [0, 1]
%   d = [P_drive; P_chg_max; v; T_amb]       traction+aux power [W] (<0 regen),
%                                            charger capability [W], speed [m/s], ambient
%   G  reality-gap struct, or [] for the pure physics twin (NMPC / EKF model).
%   The Simulink plant (build_battery_model.m) draws these equations with blocks.
if nargin < 5, G = []; end
T_c = x(1); T_s = x(2); T_w = x(3); SOC = x(4);
u_pump = u(1); u_chill = u(2); u_fan = u(3); u_rv = u(4); u_heat = u(5);
P_drive = d(1); P_chg_max = d(2); v = d(3); T_amb = d(4);
K0 = 273.15;

% thermal-system actuators
y.P_pump = P.P_pump_max * u_pump^3;
y.P_fan  = P.P_fan_max  * u_fan^3;
Q_chill_cmd = P.Q_chill_max * u_chill;
T_ev = T_w - P.dT_evap + K0;
lift = sqrt((T_amb + P.dT_cond - T_w + P.dT_evap)^2 + 1) + 5;
y.COP = 1 + P.COP_eta * T_ev / lift;              % falls as ambient rises
y.P_comp = Q_chill_cmd / y.COP;
Q_heat = P.Q_heat_max * u_heat;
y.P_heat = Q_heat;
y.P_thermal = y.P_pump + y.P_fan + y.P_comp + y.P_heat;

% BMS charge acceptance = SOC taper x temperature derate
taper = min(max((P.soc_full - SOC)/(P.soc_full - P.soc_taper), 0), 1);
derate = 1/(1 + exp((T_c - P.T_derate_hot)/1.5)) * ...
         (0.15 + 0.85/(1 + exp(-(T_c - P.T_derate_cold)/3)));
y.P_chg = P_chg_max * taper * derate;

% electrical: Joule heat with temperature- and SOC-dependent resistance
y.P_pack = P_drive + y.P_thermal - y.P_chg;      % > 0 discharge
I = y.P_pack / (P.V0 + P.V1*SOC);
R = P.R_ref * exp(P.B_R*(1/(T_c + K0) - 1/(25 + K0))) * (1 + 0.6*(SOC - 0.5)^2);
y.Q_gen = I^2 * R;

% heat paths
mdot  = P.mdot_max * u_pump;
hA_sw = P.k_sw*(abs(mdot) + 1e-3)^0.8 + P.hA_sw0;
Q_chill = Q_chill_cmd;
Q_leak  = 0;
if ~isempty(G)
    hA_sw   = hA_sw * G.plate_factor * (1 - G.plate_temp_coeff*(T_s - 25));
    Q_chill = Q_chill_cmd * (1 - G.chiller_amb_coeff*sqrt((T_amb - 25)^2 + 0.25));
    Q_leak  = G.edrive_leak * sqrt(P_drive^2 + 1e6);
end
m_air  = P.rho_a*P.A_front*(P.k_ram*v + P.v_fan_max*u_fan) + 0.05;
hA_air = P.k_air * abs(m_air)^0.8;
hA_c   = P.k_cool_rad * (abs(mdot*u_rv) + 1e-3)^0.8;
UA     = 1/(1/hA_air + 1/hA_c);
C_air  = m_air * P.cp_a;
y.Q_rad = (1 - exp(-UA/C_air)) * C_air * (T_w - T_amb);

Q_cs = P.hA_cs * (T_c - T_s);
Q_sw = hA_sw * (T_s - T_w);
xdot = [(y.Q_gen - Q_cs) / P.C_c;
        (Q_cs - Q_sw - P.hA_env*(T_s - T_amb)) / P.C_s;
        (Q_sw - Q_chill - y.Q_rad + Q_heat + Q_leak) / P.C_w;
        -y.P_pack / P.E_cap];
y.age_rate = exp(P.B_age*(1/(25 + K0) - 1/(T_c + K0)));
end
