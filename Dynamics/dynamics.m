function zdot= dynamics(t,z,d)
% z = [q1 q2 q3 q4 wx wy wz q1d q2d q3d q4d wxd wyd wzd]
% q4 is the real part of quaternion

% unpack parameters
q = z(1:4);
w = z(5:7);
qd = z(8:11);
wd = z(12:14);
invI = d.invI;
I = d.I;
T = d.T;

% normalize and properize quaternion
q = norm_prop(q);
qd = norm_prop(q);

% dynamics
q_dot = quat_prop(w,q); 
qd_dot = quat_prop(wd,qd); 
w_dot = invI*(-skew(w)*I*w + T);
wd_dot = zeros(3, 1);

% pack parameters
zdot = [q_dot; w_dot; qd_dot; wd_dot];
end