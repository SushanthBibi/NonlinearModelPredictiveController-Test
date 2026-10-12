function P = diesel_params()
%DIESEL_PARAMS  Parameters of the lumped thermal model of a ~13 L, ~400 kW
%heavy-duty diesel and its cooling system (40 t tractor-trailer).
%   Engineering estimates - replace with calibrated values when data exists.
%   SI units, temperatures in degC, powers in W.

% thermal capacities [J/K]
P.C_e = 4.0e5;        % engine block + coolant in the jacket
P.C_r = 0.8e5;        % radiator core + coolant in the radiator
P.C_o = 0.9e5;        % engine oil + sump

% fluids
P.cp_c  = 3600;       % coolant (50/50 glycol) [J/kg/K]
P.cp_a  = 1006;       % air [J/kg/K]
P.rho_a = 1.16;       % air density [kg/m^3]

% coolant pump
P.mdot_max   = 6.0;   % flow at full pump speed [kg/s]
P.P_pump_max = 3.0e3; % pump power at full speed [W]

% fan and radiator
P.A_front   = 1.2;    % radiator frontal area [m^2]
P.k_ram     = 0.30;   % share of vehicle speed seen as ram air
P.v_fan_max = 12.0;   % face velocity from fan at full speed [m/s]
P.P_fan_max = 30.0e3; % fan power at full speed [W]
P.k_air     = 728;    % air-side film:     hA = k_air  * mdot_air^0.8
P.k_cool    = 2864;   % coolant-side film: hA = k_cool * mdot_rad^0.8

% couplings
P.hA_oc0 = 1500;      % oil cooler conductance at zero flow [W/K]
P.hA_oc1 = 1000;      % extra conductance at full pump flow [W/K]
P.hA_amb = 100;       % block losses to ambient [W/K]
P.k_wall = 3000;      % liner film: hA_wall = k_wall * mdot^0.8

% engine energy split
P.eta_ind     = 0.47; % indicated efficiency
P.f_cool_fuel = 0.24; % share of fuel power into coolant
P.f_oil_fuel  = 0.03; % share of fuel power into oil
P.f_cool_fric = 0.30; % share of friction power into coolant (rest -> oil)
P.LHV         = 42.8e6;

% friction: P_fric = P_fric_ref*(N/1000)^1.5*(0.5 + 0.5*exp(B*(1/T_o - 1/T_ref)))
P.P_fric_ref = 6.5e3;
P.visc_B     = 1500;
P.T_oil_ref  = 100;

% limits
P.T_e_max     = 102;  % coolant
P.T_o_max     = 125;  % oil
P.T_metal_max = 130;  % cylinder liner
P.u_pump_min  = 0.12;

% conventional cooling system (baseline controller)
P.T_open      = 82;   % wax thermostat starts to open
P.T_full      = 95;   % wax thermostat fully open
P.fan_on      = 97;   % viscous fan engages
P.fan_off     = 92;   % viscous fan disengages
P.N_pump_full = 1800; % engine speed for full mechanical pump speed [rpm]
end
