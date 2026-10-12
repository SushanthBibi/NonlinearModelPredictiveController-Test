function xn = battery_rk4(x, u, d, dt, P, G)
%BATTERY_RK4  One Runge-Kutta 4 step of battery_model.
if nargin < 6, G = []; end
k1 = battery_model(x,           u, d, P, G);
k2 = battery_model(x + dt/2*k1, u, d, P, G);
k3 = battery_model(x + dt/2*k2, u, d, P, G);
k4 = battery_model(x + dt*k3,   u, d, P, G);
xn = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
