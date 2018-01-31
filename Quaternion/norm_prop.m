function q_new = norm_prop(q)
% normalize and properize quaternion
% q = -q
q_new = q./norm(q);
% if q_new(4) < 0
%     q_new = -q_new;
% end
end