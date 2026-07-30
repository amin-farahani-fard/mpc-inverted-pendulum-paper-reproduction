function model = build_robot_model(cfg)
% Modele 6-halate robot
% Halat: [fasele; jahat; tilt; sorat; nerkhe yaw; nerkhe tilt]
% Voroodi: [torque rast; torque chap]

p = cfg.plant;
n = p.gearRatio;
r = p.rw;
b = p.b;

% Matrise jerm va jazebe dar model linear
m11 = p.mb + 2*(p.Iwy + n^2*p.Iry)/r^2;
m22 = p.Izz + (p.Iwz + p.Irz) + 2*(b/r)^2*(p.Iwy + n^2*p.Iry);
m13 = 2*(n - n^2)*p.Iry/r^2;
m33 = p.mb*p.l^2 + p.Iyy + 2*((1 + n^2 - n)*p.Iry - p.Iwy);

M = [m11 0 m13; 0 m22 0; m13 0 m33];
G = diag([0 0 -p.mb*p.g*p.l]);
E = [1/r 1/r; b/r -b/r; -1 -1];

Ac = [zeros(3) eye(3); -M\G zeros(3)];
Bc = [zeros(3,2); M\E];
Cc = eye(6);
Dc = zeros(6,2);

sysc = ss(Ac, Bc, Cc, Dc);
sys1 = c2d(sysc, cfg.sample.delta1, 'zoh');
[Fd,~,lqPoles] = dlqr(sys1.A, sys1.B, cfg.lq.Q, cfg.lq.R);

% Lift chand-nerkhe daghigh baraye halghe digital LQ
nLift = cfg.sample.multirate;
Acl1 = sys1.A-sys1.B*Fd;
A2 = Acl1^nLift;
B2 = zeros(size(sys1.B));
for j=0:nLift-1
    B2 = B2 + Acl1^j*sys1.B;
end

model.M = M;
model.G = G;
model.E = E;
model.Ac = Ac;
model.Bc = Bc;
model.Ad1 = sys1.A;
model.Bd1 = sys1.B;
model.Fd = Fd;
model.lqPoles = lqPoles;
model.Adf1 = Acl1;
model.Adf2 = A2;
model.Bdf2 = B2;
model.stateNames = {'distance','direction','tilt','velocity','yaw_rate','tilt_rate'};
model.inputNames = {'right_torque','left_torque'};
end
