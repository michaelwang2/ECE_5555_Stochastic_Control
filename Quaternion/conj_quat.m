function q_conj = conj_quat(q)
% calculates the conjugate of a quaternion
q_conj = [-q(1:3); q(4)];
end