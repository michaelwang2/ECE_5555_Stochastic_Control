function z1 = RK4(odefun, t, z, p, dt)
k1 = odefun(t, z, p); 
k2 = odefun(t + dt/2, z + dt*k1/2, p);
k3 = odefun(t + dt/2, z + dt*k2/2, p);
k4 = odefun(t + dt, z + dt*k3, p);
z1 = z + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
end