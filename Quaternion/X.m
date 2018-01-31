function X_quat = X(q)
% A(q) = E(q)'*X(q)
X_quat = [q(4).*eye(3) - skew(q(1:3)); -q(1:3).'];
end