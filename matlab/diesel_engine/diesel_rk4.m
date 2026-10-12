function xn = diesel_rk4(x, u, d, dt, P)
%DIESEL_RK4  One Runge-Kutta 4 step of diesel_model.
k1 = diesel_model(x,             u, d, P);
k2 = diesel_model(x + dt/2*k1,   u, d, P);
k3 = diesel_model(x + dt/2*k2,   u, d, P);
k4 = diesel_model(x + dt*k3,     u, d, P);
xn = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
end
