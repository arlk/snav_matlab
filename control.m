function [f, rate] = control(ref, pose, origin, t, dt)
% Params
m = 0.394;
g = 9.81;
kx = 5.0; %[5.0 5.0 5.0]';
kv = 2.5; %[2.5 2.5 2.5]';
kR = 0.5; %[1.5 1.5 1.5]';

% Fixed Vars
e3 = [0.0 0.0 1.0]';
R = pose.rot;

% Force
ex = ref.pos - pose.pos;
ev = ref.vel - pose.vel;
f_des = kx.*ex + kv.*ev + m*g.*e3 + m.*ref.acc;
f = f_des'*R(:,end);

% Rates

% FIXME: Add feedforward terms
% omega_di = veeMap(Rc'*Rc_dot);
% rate = R_tilde'*omega_di - kR.*eR;
b3d = f_des./norm(f_des);
yawAux = [cos(ref.yaw) sin(ref.yaw) 0]';
yawAux = cross(b3d,yawAux);
b2d = yawAux./norm(yawAux);
b1d = cross(b2d,b3d);
Rc = [b1d b2d b3d];

Rt = Rc'*R;
eR = 0.5.*veeMap(Rt - Rt');
rate = -kR.*eR;
end

