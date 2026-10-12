function [P, G] = battery_params()
%BATTERY_PARAMS  540 kWh heavy-duty e-truck pack and its thermal loop.
%   P  physics parameters (what the digital twin / NMPC model uses)
%   G  "reality gap": effects present in the simulated real pack that the
%      twin does not model (set all to neutral values to remove them)
%   Engineering estimates. SI units, temperatures in degC, powers in W.

% electrical
P.E_cap = 540e3*3600;   % usable energy [J]
P.V0    = 560;          % open-circuit voltage at SOC = 0 [V]
P.V1    = 180;          % OCV slope [V per unit SOC]
P.R_ref = 0.035;        % pack resistance at 25 degC, SOC 0.5 [ohm]
P.B_R   = 3500;         % Arrhenius sensitivity of resistance [K]

% thermal capacities [J/K]
P.C_c = 2.4e6;          % cell cores
P.C_s = 0.6e6;          % cell surfaces + module structure + cold plates
P.C_w = 0.3e6;          % coolant loop

% conductances [W/K]
P.hA_cs  = 4000;        % core -> surface
P.k_sw   = 1720;        % surface -> coolant: k_sw*mdot^0.8 + hA_sw0
P.hA_sw0 = 150;
P.hA_env = 60;          % enclosure -> ambient

% coolant pump
P.mdot_max   = 2.0;     % [kg/s]
P.P_pump_max = 0.8e3;
P.u_pump_min = 0.05;

% chiller (refrigerant loop)
P.Q_chill_max = 30e3;   % cooling capacity [W]
P.COP_eta     = 0.40;   % fraction of Carnot COP
P.dT_evap     = 8;      % evaporator approach [K]
P.dT_cond     = 12;     % condenser approach [K]

% low-temperature radiator + fan
P.A_front    = 0.6;
P.k_ram      = 0.25;
P.v_fan_max  = 10;
P.P_fan_max  = 2.5e3;
P.rho_a      = 1.16;
P.cp_a       = 1006;
P.k_air      = 500;
P.k_cool_rad = 2500;

% PTC heater
P.Q_heat_max = 12e3;

% BMS charging behaviour
P.soc_taper     = 0.80; % charge power tapers linearly above this SOC
P.soc_full      = 0.97;
P.T_derate_hot  = 43;   % 50 % charge derate point, hot
P.T_derate_cold = 12;   % 50 % charge derate point, cold (lithium plating)

% limits and targets
P.T_max     = 50;       % absolute core limit
P.T_band_hi = 38;       % preferred window
P.T_band_lo = 18;

% ageing (Arrhenius, 1 at 25 degC)
P.B_age = 3700;

% rule-based BMS thresholds (baseline controller)
P.chill_on = 35; P.chill_off = 31;   % cell surface temperature
P.heat_on  = 10; P.heat_off  = 15;

% ---- reality gap (truth only)
G.plate_factor      = 0.75;   % cold-plate conductance 25 % lower (TIM ageing)
G.plate_temp_coeff  = 0.006;  % ... and a further 0.6 %/K lower above 25 degC
G.chiller_amb_coeff = 0.012;  % chiller capacity -1.2 %/K away from 25 degC ambient
G.edrive_leak       = 0.012;  % 1.2 % of traction power leaks into the battery loop
end
