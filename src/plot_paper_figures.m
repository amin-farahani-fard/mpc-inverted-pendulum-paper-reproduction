function plot_paper_figures(result,cfg,outputDir)


t=result.time;
spec = { ...
    'Fig07_displacement', result.distance, 'Distance [m]', 'Distance', [0 105]; ...
    'Fig08_direction', result.directionDeg, 'Angle [deg]', 'Direction', [-2 4]; ...
    'Fig09_tilt', result.tiltDeg, 'Angle [deg]', 'Tilt angle', [-40 40]; ...
    'Fig10_velocity', result.velocity, 'Velocity [m/s]', 'Velocity', [-0.5 4]};

for i=1:size(spec,1)
    f=figure('Color','w','Position',[100 100 620 340]);
    ax=axes(f); hold(ax,'on');
    plot(ax,t,spec{i,2},'LineWidth',1.5,'Color',[0 0.4470 0.7410]);
    grid on;
    box on; 
    xlim([0 cfg.simulation.stopTime]);
    ylim(spec{i,5});
    style_axes(ax);
    xlabel(ax,'Time[s]'); ylabel(ax,spec{i,3});
    if i==3
        yline(ax,30,'--k','HandleVisibility','off');
        yline(ax,-30,'--k','HandleVisibility','off');
    end
    lgd=legend(ax,{spec{i,4}},'Location','northeast');
    lgd.AutoUpdate='off';
    style_legend(lgd);
    exportgraphics(f,fullfile(outputDir,[spec{i,1} '.png']), ...
        'Resolution',200,'BackgroundColor','white');
    savefig(f,fullfile(outputDir,[spec{i,1} '.fig']));
end

f=figure('Color','w','Position',[100 100 620 340]);
ax=axes(f); hold(ax,'on');
plot(ax,t,result.rightTorque,'LineWidth',1.2,'Color',[0 0.4470 0.7410]);
plot(ax,t,result.leftTorque,'--k','LineWidth',1.2);
yline(cfg.constraints.torque,'--','Color',[.55 .55 .55]);
yline(-cfg.constraints.torque,'--','Color',[.55 .55 .55]);
grid on; 
box on; 
xlim([0 cfg.simulation.stopTime]); ylim([-1.5 1.5]);
style_axes(ax);
xlabel(ax,'Time[s]'); ylabel(ax,'Torque [Nm]');
lgd=legend(ax,{'Right torque','Left torque'},'Location','northeast');
style_legend(lgd);
exportgraphics(f,fullfile(outputDir,'control_input.png'), 'Resolution',200,'BackgroundColor','white');
savefig(f,fullfile(outputDir,'control_input.fig'));
end

function style_axes(ax)
% Rang sabet baraye khoroojie yekdast
set(ax,'Color','white','XColor','black','YColor','black', ...
    'GridColor',[0.82 0.82 0.82],'GridAlpha',0.65, ...
    'FontName','Times New Roman','FontSize',11,'LineWidth',0.75);
end

function style_legend(lgd)
set(lgd,'Color','white','TextColor','black','EdgeColor',[0.25 0.25 0.25], ...
    'FontName','Times New Roman','FontSize',11,'Box','on');
end
