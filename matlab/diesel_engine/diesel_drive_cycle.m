function cyc = diesel_drive_cycle(T_amb)
%DIESEL_DRIVE_CYCLE  Synthetic 36-min heavy-duty truck mission, 1 s samples.
%   Engine brake power and speed follow from longitudinal vehicle dynamics.
if nargin < 1, T_amb = 25; end

% [duration s, target speed m/s, grade -]
seg = [ 60   0   0;      % cold start, idle
       240  12   0;      % urban
        60   0   0;      % stop
       420  22   0;      % highway
       300  16   0.045;  % long climb
       180  20  -0.04;   % descent (engine braking)
       360  22   0;      % highway
       240  15   0.05;   % second climb
       240  22   0;      % highway
        60   0   0];     % stop

m = 40000; CdA = 6.0; Crr = 0.006; rho = 1.2; eta_dl = 0.92;
P_max = 380e3; P_idle = 3e3; g = 9.81; a_max = 0.4;

target = repelem(seg(:,2), seg(:,1));
grade  = repelem(seg(:,3), seg(:,1));
n = numel(target);
v = zeros(n,1);
for k = 2:n                                   % rate-limited speed
    dv   = min(max(target(k) - v(k-1), -1.5*a_max), a_max);
    v(k) = max(v(k-1) + dv, 0);
end
a = gradient(v, 1);
F = m*a + 0.5*rho*CdA*v.^2 + m*g*Crr*(v > 0.1) + m*g*sin(atan(grade));
P_wheel = F .* v;
P_b = P_wheel / eta_dl;
P_b(P_wheel < 0) = P_wheel(P_wheel < 0) * eta_dl;
P_b = min(max(P_b, -150e3), P_max) + P_idle;

N = min(max(850 + 20*v, 900), 1500);          % crude gear map [rpm]
N(v < 0.5) = 600;
N = N + 250*(grade > 0.02);

cyc.t = (0:n-1)';
cyc.v = v; cyc.grade = grade; cyc.N = N; cyc.P_b = P_b;
cyc.T_amb = T_amb*ones(n,1);
cyc.D = [N P_b v cyc.T_amb]';                 % 4 x n disturbance matrix
end
