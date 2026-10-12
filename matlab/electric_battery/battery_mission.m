function M = battery_mission(T_amb)
%BATTERY_MISSION  Regional haul: 80 min driving -> 30 min megawatt charging
%during the driver's break -> 40 min driving. 1 s samples.
if nargin < 1, T_amb = 25; end
% [duration s, speed m/s, grade -, charger power W]
seg = [ 300 14  0      0;   1200 22  0      0;    600 17  0.035  0;
        420 22 -0.03   0;    900 22  0      0;    480 15  0.045  0;
        300 22 -0.035  0;    600 22  0      0;
       1800  0  0  750e3;                                    % MCS charging stop
        900 22  0      0;    540 16  0.04   0;    360 22 -0.03   0;
        600 22  0      0];
mass = 40000; CdA = 5.5; Crr = 0.0055; rho = 1.2; g = 9.81;
eta_drive = 0.90; eta_regen = 0.60; aux = 5e3; P_trac_max = 480e3; P_regen_max = 250e3;

v_t   = repelem(seg(:,2), seg(:,1));
grade = repelem(seg(:,3), seg(:,1));
chg   = repelem(seg(:,4), seg(:,1));
n = numel(v_t);
v = v_t;
for k = 2:n                                  % first-order speed smoothing (~20 s)
    v(k) = v(k-1) + (v_t(k) - v(k-1))/20;
end
a = gradient(v, 1);
F = mass*a + 0.5*rho*CdA*v.^2 + mass*g*(Crr*(v > 0.5) + grade);
P_wheel = F .* v;
P_drive = min(P_wheel/eta_drive, P_trac_max);
neg = P_wheel <= 0;
P_drive(neg) = max(P_wheel(neg)*eta_regen, -P_regen_max);
P_drive = P_drive + aux;

M.t = (0:n-1)';
M.v = v; M.grade = grade; M.P_drive = P_drive; M.P_chg_max = chg;
M.T_amb = T_amb*ones(n,1);
M.D = [P_drive chg v M.T_amb]';              % 4 x n disturbance matrix
idx = find(chg > 0);
M.charge_window = [M.t(idx(1)), M.t(idx(end))];
end
