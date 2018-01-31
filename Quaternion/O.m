function O_quat = O(w)
% w = column vector angular velocity
% q_dot = 0.5*O(w)*q
O_quat = [-skew(w), w; -w.', 0];
end