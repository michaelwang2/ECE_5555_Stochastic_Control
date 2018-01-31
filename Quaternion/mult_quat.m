function q_mult = mult_quat(p,q)
% quaternion multiplcation p x q
% q = [v; q4] 
% - v = [q1; q2; q3] = e.*sin(theta/2) 
% - q4 = cos(theta/2)
% q_mult = zeros(4,1);
% q_mult(4) = p(4)*q(4) - p(1:3)'*q(1:3);
% q_mult(1:3) = p(4).*q(1:3) + q(4).*p(1:3) - skew(p(1:3))*q(1:3);

% left-handed convention (currently using) 
q_mult = [p(4).*q(1:3) + q(4).*p(1:3) - skew(p(1:3))*q(1:3); ...
    p(4)*q(4) - p(1:3)'*q(1:3)];

% % right-handed
% q_mult = [p(4).*q(1:3) + q(4).*p(1:3) + skew(p(1:3))*q(1:3); ...
%     p(4)*q(4) - p(1:3)'*q(1:3)];
end