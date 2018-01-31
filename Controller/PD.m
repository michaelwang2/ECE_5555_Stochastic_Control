function T = PD(q_hat, w_hat, qd, wd, I, D, K)
% Nonlinear PD control
dq = mult_quat(q_hat, conj_quat(qd));
T = skew(w_hat)*I*w_hat + D*(wd - w_hat) - K*(dq(1:3));
end