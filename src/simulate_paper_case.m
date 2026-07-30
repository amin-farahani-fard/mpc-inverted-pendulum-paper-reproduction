function result = simulate_paper_case(cfg, model)
% Shabihsazie chand-nerkhe LQ/MPC

dt = cfg.sample.delta1;
t = (0:dt:cfg.simulation.stopTime).';
N = numel(t);

x = zeros(N,6);
uActual = zeros(N,2);
uMpcHistory = zeros(N,2);
velocityLimit = zeros(N,1);
solverExit = nan(N,1);
tiltSlack = nan(N,1);
velocitySlack = nan(N,1);
solverMode = nan(N,1);
x(1,:) = cfg.simulation.x0.';
uMpc = cfg.simulation.uMpc0;

for k = 1:N-1
    if mod(k-1,cfg.sample.multirate) == 0
        ref = make_reference_horizon(t(k), cfg);
        [uMpcNew, info] = solve_multirate_mpc(x(k,:).', uMpc, ref, cfg, model);
        if info.exitflag > 0
            uMpc = uMpcNew;
        end
        solverExit(k) = info.exitflag;
        tiltSlack(k) = info.tiltSlack;
        velocitySlack(k) = info.velocitySlack;
        solverMode(k) = info.solveMode;
        velocityLimit(k) = info.velocityLimit;
    elseif k > 1
        velocityLimit(k) = velocityLimit(k-1);
    end

    % Farmane motor: MPC menhaye feedbacke LQ
    tau = uMpc - model.Fd*x(k,:).';
    uActual(k,:) = tau.';
    uMpcHistory(k,:) = uMpc.';
    x(k+1,:) = (model.Ad1*x(k,:).' + model.Bd1*tau).';
end

uActual(end,:) = uActual(end-1,:);
uMpcHistory(end,:) = uMpcHistory(end-1,:);
velocityLimit(end) = velocityLimit(end-1);

result.time = t;
result.state = x;
result.distance = x(:,1);
result.directionDeg = rad2deg(x(:,2));
result.tiltDeg = rad2deg(x(:,3));
result.velocity = x(:,4);
result.rightTorque = uActual(:,1);
result.leftTorque = uActual(:,2);
result.uMpc = uMpcHistory;
result.velocityLimit = velocityLimit;
result.solverExit = solverExit;
result.tiltSlack = tiltSlack;
result.velocitySlack = velocitySlack;
result.solverMode = solverMode;
end
