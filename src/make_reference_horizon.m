function Xref = make_reference_horizon(t0, cfg)
% Marjae halat dar ofoghe pishbini

Hp = cfg.mpc.Hp;
Xref = zeros(6,Hp);
for i = 1:Hp
    ti = t0 + i*cfg.sample.delta2;
    Xref(1,i) = cfg.simulation.destination;
    Xref(2,i) = heading_reference(ti, cfg.trajectory.headingEvents);
end
Xref = Xref(:);
end

function phi = heading_reference(t, events)
phi = 0;
for i = 1:size(events,1)
    if t >= events(i,1) && t < events(i,2)
        phi = events(i,3);
        return;
    end
end
end
