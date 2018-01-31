function [tarray, zarray] = leapfrog(odefun,tspan,z0,n,p)
% this implements the velocity verlet (leapfrog) integrator
% z = [x;v];
tarray = linspace(tspan(1),tspan(2),n+1);
dt = tarray(2) - tarray(1);
zarray = zeros(n+1,length(z0));
zarray(1,:) = z0';
num = length(z0)/2;

for i = 1:n
    v = zarray(i,(end-num+1):end)';
    x = zarray(i,1:num)';
    z = [x;v];
    Fn = odefun(tarray(i),z,p);
    
    v_1_2 = v + 0.5*dt.*Fn((end-num+1):end);
    xn1 = x + dt.*v_1_2;
    zarray(i+1,1:num) = xn1';
    z = [xn1; v_1_2];
    Fn1 = odefun(tarray(i),z,p);
    
    zarray(i+1,(end-num+1):end) = (v_1_2 + 0.5*dt.*Fn1((end-num+1):end))';
end
end