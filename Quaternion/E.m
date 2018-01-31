function E_quat = E(q)
% q = [v; q4] 
% q_dot = 0.5*E(q)*w
E_quat = [q(4).*eye(3) + skew(q(1:3)); -q(1:3).'];
end