function [xdot, y] = diesel_model(x, u, d, P)
%DIESEL_MODEL  Nonlinear thermal model of engine + cooling system.
%   x = [T_e; T_r; T_o]          engine/coolant, radiator, oil  [degC]
%   u = [u_pump; u_fan; u_th]    pump speed, fan speed, thermostat valve [0..1]
%   d = [N; P_b; v; T_amb]       engine speed [rpm], brake power [W],
%                                vehicle speed [m/s], ambient [degC]
%   The Simulink plant (build_diesel_model.m) draws exactly these equations
%   with blocks; this function is the NMPC's internal prediction model.

T_e = x(1); T_r = x(2); T_o = x(3);
u_pump = u(1); u_fan = u(2); u_th = u(3);
N = d(1); P_b = d(2); v = d(3); T_amb = d(4);

% actuator power (affinity laws: power ~ speed^3)
y.P_pump = P.P_pump_max * u_pump^3;
y.P_fan  = P.P_fan_max  * u_fan^3;

% oil-temperature dependent friction
phi = exp(P.visc_B * (1/(T_o + 273.15) - 1/(P.T_oil_ref + 273.15)));
y.P_fric = P.P_fric_ref * (N/1000)^1.5 * (0.5 + 0.5*phi);

% fuel: friction and auxiliaries come off the crank (smooth max(.,0) = no fuel when braking)
P_crank = P_b + y.P_fric + y.P_pump + y.P_fan;
P_ind   = 0.5 * (P_crank + sqrt(P_crank^2 + 1e6));
y.P_fuel    = P_ind / P.eta_ind;
y.mdot_fuel = y.P_fuel / P.LHV;

% heat sources
y.Q_cool = P.f_cool_fuel*y.P_fuel + P.f_cool_fric*y.P_fric;
Q_oil    = P.f_oil_fuel*y.P_fuel + (1 - P.f_cool_fric)*y.P_fric;

% coolant flows
mdot     = P.mdot_max * u_pump;
mdot_rad = mdot * u_th;

% radiator: air-side effectiveness, series air/coolant films
m_air = P.rho_a*P.A_front*(P.k_ram*v + P.v_fan_max*u_fan) + 0.05;
hA_air  = P.k_air  * abs(m_air)^0.8;
hA_cool = P.k_cool * (abs(mdot_rad) + 1e-3)^0.8;
UA    = 1 / (1/hA_air + 1/hA_cool);
C_air = m_air * P.cp_a;
y.Q_rad = (1 - exp(-UA/C_air)) * C_air * (T_r - T_amb);

% couplings
Q_oc   = (P.hA_oc0 + P.hA_oc1*u_pump) * (T_o - T_e);
Q_adv  = mdot_rad * P.cp_c * (T_e - T_r);
Q_loss = P.hA_amb * (T_e - T_amb);

xdot = [(y.Q_cool + Q_oc - Q_adv - Q_loss) / P.C_e;
        (Q_adv - y.Q_rad) / P.C_r;
        (Q_oil - Q_oc) / P.C_o];

% liner metal temperature: film drop shrinks with coolant flow
y.T_metal = T_e + y.Q_cool / (P.k_wall * (abs(mdot) + 1e-3)^0.8);
end
