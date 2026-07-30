function [uNext, info] = solve_multirate_mpc(x0, uPrevious, Xref, cfg, model)
% MPC ba gheyde sakht motor va gheyde narme halat

A = model.Adf2;
B = model.Bdf2;
Fd = model.Fd;
nx = size(A,1);
nu = size(B,2);
Hp = cfg.mpc.Hp;
Hu = cfg.mpc.Hu;

[Psi, Gamma, Theta, U0, Tu] = prediction_matrices(A, B, Hp, Hu);
freeX = Psi*x0 + Gamma*uPrevious;

Qbar = kron(eye(Hp),cfg.mpc.Qstage);
idxLast = (Hp-1)*nx + (1:nx);
Qbar(idxLast,idxLast) = cfg.mpc.terminalMultiplier*cfg.mpc.Qstage;
Rbar = kron(eye(Hu),cfg.mpc.Rmove);

H = 2*(Theta.'*Qbar*Theta + Rbar);
H = (H+H.')/2;
f = 2*Theta.'*Qbar*(freeX-Xref);

% Pishbinie torque vagheyi
Ufree = U0*uPrevious;
Fstack = kron(eye(Hp),Fd);
actualFree = Ufree - Fstack*freeX;
actualMap = Tu - Fstack*Theta;

tauMax = min(cfg.constraints.torque, ...
    cfg.plant.tauStall-cfg.constraints.torqueSafetyMargin);
Dfirst=zeros(nu,Hu*nu);
Dfirst(:,1:nu)=eye(nu);
currentActualFree=uPrevious-Fd*x0;
Aineq = [Dfirst; -Dfirst; actualMap; -actualMap];
bineq = [tauMax*ones(nu,1)-currentActualFree; ...
         tauMax*ones(nu,1)+currentActualFree; ...
         tauMax*ones(Hp*nu,1)-actualFree; ...
         tauMax*ones(Hp*nu,1)+actualFree];

vmax = distance_velocity_limit(x0(1),cfg);

% Gheyde halat dar tamame ofoghe pishbini
[As,bs] = state_constraints(freeX,Theta,Hp,nx,vmax,cfg);
AineqNominal = [Aineq; As];
bineqNominal = [bineq; bs];

duMax = cfg.constraints.deltaU;
lb = -duMax*ones(Hu*nu,1);
ub =  duMax*ones(Hu*nu,1);
options = optimoptions('quadprog', ...
    'Display','off', ...
    'Algorithm',cfg.solver.algorithm, ...
    'MaxIterations',cfg.solver.maxIterations, ...
    'ConstraintTolerance',cfg.solver.constraintTolerance, ...
    'OptimalityTolerance',cfg.solver.optimalityTolerance);

[dU,~,exitflag] = quadprog(H,f,AineqNominal,bineqNominal,[],[],lb,ub,[],options);
relaxed = false;
stateSlack = zeros(2,1);
solveMode = 0; % 0 adi, 1 narm, 2 ezterari, 3 faghat motor
if exitflag <= 0
    % Gheyde narm ba slack jodagane baraye har game pishbini
    nMove=Hu*nu;
    nSlack=2*Hp;
    objectiveScale=max([1,norm(H,inf),norm(f,inf)]);
    Hnormalized=H/objectiveScale;
    fnormalized=f/objectiveScale;
    rho=cfg.solver.stateSlackPenalty;
    slackPenalty=[rho*ones(Hp,1); ...
        rho*cfg.solver.velocitySlackPenaltyMultiplier*ones(Hp,1)];
    Hsoft=blkdiag(Hnormalized,2*diag(slackPenalty));
    fsoft=[fnormalized;zeros(nSlack,1)];
    AactSoft=[Aineq zeros(size(Aineq,1),nSlack)];
    slackMap=zeros(4*Hp,nSlack);
    slackMap(1:Hp,1:Hp)=-eye(Hp);
    slackMap(Hp+1:2*Hp,1:Hp)=-eye(Hp);
    slackMap(2*Hp+1:3*Hp,Hp+1:2*Hp)=-eye(Hp);
    slackMap(3*Hp+1:4*Hp,Hp+1:2*Hp)=-eye(Hp);
    AstateSoft=[As slackMap];
    lbSoft=[lb;zeros(nSlack,1)];
    ubSoft=[ub; ...
        (cfg.constraints.relaxedTiltFactor-1)*cfg.constraints.tilt*ones(Hp,1); ...
        (cfg.constraints.relaxedVelocityFactor-1)*vmax*ones(Hp,1)];
    [z,~,exitflag] = quadprog(Hsoft,fsoft,[AactSoft;AstateSoft], ...
        [bineq;bs],[],[],lbSoft,ubSoft,[],options);
    if ~isempty(z)
        dU=z(1:nMove);
        slackVector=z(nMove+(1:nSlack));
        stateSlack=[max(slackVector(1:Hp));max(slackVector(Hp+1:end))];
    else
        dU=[];
    end
    relaxed=any(stateSlack>1e-9);
    if exitflag>0; solveMode=1; end
end

% Slack ezterari dar soorate infeasible boodan
if exitflag <= 0
    rhoEmergency=cfg.solver.emergencySlackPenalty;
    emergencyPenalty=[rhoEmergency*ones(Hp,1); ...
        rhoEmergency*cfg.solver.velocitySlackPenaltyMultiplier*ones(Hp,1)];
    Hemergency=blkdiag(Hnormalized,2*diag(emergencyPenalty));
    ubEmergency=[ub;inf(nSlack,1)];
    [z,~,exitflag] = quadprog(Hemergency,fsoft,[AactSoft;AstateSoft], ...
        [bineq;bs],[],[],lbSoft,ubEmergency,[],options);
    if ~isempty(z)
        dU=z(1:nMove);
        slackVector=z(nMove+(1:nSlack));
        stateSlack=[max(slackVector(1:Hp));max(slackVector(Hp+1:end))];
        relaxed=any(stateSlack>1e-9);
    else
        dU=[];
    end
    if exitflag>0; solveMode=2; end
end

% Akharin rah: hefze gheyde sakht motor
if exitflag <= 0
    [dU,~,exitflag] = quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],options);
    solveMode=3;
end

if isempty(dU)
    uNext = uPrevious;
else
    uNext = uPrevious + dU(1:nu);
end

% Eemale nahayie gheyde torque
tauNow=uNext-Fd*x0;
tauNow=min(max(tauNow,-tauMax),tauMax);
uNext=tauNow+Fd*x0;

info.exitflag = exitflag;
info.relaxed = relaxed;
info.velocityLimit = vmax;
info.tiltSlack = stateSlack(1);
info.velocitySlack = stateSlack(2);
info.solveMode = solveMode;
end

function [As,bs] = state_constraints(freeX,Theta,Hp,nx,vmax,cfg)
tiltMax = cfg.constraints.tilt;

rowTilt = zeros(Hp,Hp*nx);
rowVel = zeros(Hp,Hp*nx);
for i=1:Hp
    rowTilt(i,(i-1)*nx+3) = 1;
    rowVel(i,(i-1)*nx+4) = 1;
end
C = [rowTilt; -rowTilt; rowVel; -rowVel];
limits = [tiltMax*ones(2*Hp,1); vmax*ones(2*Hp,1)];
As = C*Theta;
bs = limits-C*freeX;
end

function vmax = distance_velocity_limit(distance,cfg)
breaks = cfg.constraints.distanceBreaks;
distance=max(distance,breaks(1));
index = find(distance >= breaks(1:end-1) & distance < breaks(2:end),1,'first');
if isempty(index); index=numel(cfg.constraints.velocity); end
vmax = cfg.constraints.velocity(index);
end
