clear;
clc; 
close all;

projectRoot = fileparts(mfilename('fullpath'));
addpath(fullfile(projectRoot,'src'),'-begin');

cfg = paper_parameters();
model = build_robot_model(cfg);
result = simulate_paper_case(cfg, model);

outputDir = fullfile(projectRoot,'output');
if ~exist(outputDir, 'dir'); mkdir(outputDir); end
save(fullfile(outputDir, 'simulation_result.mat'), 'result', 'cfg', 'model');

plot_paper_figures(result, cfg, outputDir);

