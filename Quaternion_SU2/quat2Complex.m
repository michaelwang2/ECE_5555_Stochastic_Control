function C = quat2Complex(q)
% q = [v; q4] = [b c d a]
C = [q(4) + q(1)*i, q(2) + q(3)*i; -q(2) + q(3)*i, q(4) - q(1)*i];
end