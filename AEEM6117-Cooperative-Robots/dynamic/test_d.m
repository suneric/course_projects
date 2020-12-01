clear
clc % clean command window

warning('off','all')
rng('shuffle');

% define room
% [roomx_min, roomx_max, roomy_min, roomy_max, door_width, door_dpeth]
room = [-1 1 -1 0 0.2 0.2];

% predefine the mass and length of the rod
m = 1;
l = 0.5;

% load best fit parameters and rebuild the fuzzy controllers
fis1 = readfis('robot_d');
fis2 = readfis('robot_d');
params = load('./BestGenes/Gen100.mat');
[fis1, fis2] = LoadFuzzyControllers_d(params.R, fis1, fis2);

fis = readfis('robot');
singleFisParams = load('model_single_r100.mat');
fis = LoadFuzzyController(singleFisParams.R, fis);

count = 10;
%testSamples = CreateSamples(l, room, count);
testSamples = CreateRandomSamples(l, room, count);

deltaT = 0.01;
maxStep = 500;
failure = 0;
for i = 1:count
[cost, step, success, trajectory] = CreateTrajectory_d(testSamples(i,:), room, m, l, fis, fis1, fis2, deltaT, maxStep);
PlotTrajectory(trajectory, room, l, deltaT, true);
fprintf('%d th: success: %d \t total distance: %.3f \t step: %d \n',i, success, cost, step);
   if ~success
       failure = failure + 1;
   end
end
fprintf('%d failure / %d total \n', failure, count);
