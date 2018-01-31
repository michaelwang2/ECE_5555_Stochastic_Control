function q_dot = quat_prop(w,q) 
% calcualtes q_dot from quaternion kinematics
% angular velocity vector is in body frame
q_dot = (0.5).*mult_quat([w;0],q);
end