function cfg = paper_parameters()
% Parameterhaye model va controller

% Moshakhasate mekaniki (Jadvale 1)
p.l = 0.040;                  % fasele markaze jerm [m]
p.track = 0.33;               % fasele charkha [m]
p.b = p.track/2;
p.dp = 0.15;                  % omghe badane [m]
p.rw = 0.08;                  % shoae charkh [m]
p.mb = 3.50;                  % jerme badane [kg]
p.mw = 0.25;                  % jerme har charkh [kg]
p.Ixx = 1.752e-1;
p.Iyy = 0.321e-1;
p.Izz = 1.473e-1;
p.Iwx = 4.00e-4;
p.Iwz = 4.00e-4;
p.Iwy = 8.00e-4;
p.Irx = 1.419e-4;
p.Irz = 1.419e-4;
p.Iry = 3.375e-5;
p.tauStall = 1.1;
p.gearRatio = 1/9;
p.g = 9.80665;
cfg.plant = p;

% Tanzimate namunebardari va controller
cfg.sample.delta1 = 0.01;
cfg.sample.delta2 = 0.08;
cfg.sample.multirate = 8;

cfg.lq.Q = diag([1e4 1e2 1e2 1e2 1e2 1e3]);
cfg.lq.R = diag([1e5 1e5]);

cfg.mpc.Qstage = diag([1e2 1e2 1e3 1e2 1e1 1e3]);
cfg.mpc.terminalMultiplier = 3;
cfg.mpc.Rmove = eye(2);
cfg.mpc.Hp = 10;
cfg.mpc.Hu = 5;
cfg.mpc.Hw = 1;

% Gheyde sorat, torque va zavie
cfg.constraints.distanceBreaks = [0 8 20 35 60 inf];
cfg.constraints.velocity = [1.1 3.3 2.2 3.3 2.7];
cfg.constraints.deltaU = 2.18;
cfg.constraints.torque = cfg.constraints.deltaU/2;
cfg.constraints.tilt = deg2rad(30);
cfg.constraints.relaxedVelocityFactor = 3.0;
cfg.constraints.relaxedTiltFactor = 1.2;
cfg.constraints.torqueSafetyMargin = 0.01;

cfg.simulation.stopTime = 60;
cfg.simulation.x0 = zeros(6,1);
cfg.simulation.uMpc0 = zeros(2,1);
cfg.simulation.destination = 100;

% Pulsehaye marjae jahate harekat
cfg.trajectory.headingEvents = [ ...
    12.57, 12.97, deg2rad(8.6); ...
    12.97, 13.45, deg2rad(-5.4); ...
    26.54, 26.94, deg2rad(2.1); ...
    26.94, 27.42, deg2rad(-1.3)];

cfg.solver.algorithm = 'interior-point-convex';
cfg.solver.maxIterations = 200;
cfg.solver.constraintTolerance = 1e-8;
cfg.solver.optimalityTolerance = 1e-9;
cfg.solver.stateSlackPenalty = 1e4;
cfg.solver.velocitySlackPenaltyMultiplier = 1;
cfg.solver.emergencySlackPenalty = 1e5;
end
