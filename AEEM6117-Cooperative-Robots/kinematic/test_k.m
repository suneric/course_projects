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

testSamples = RandomTestSamples();
%PlotSamples(testSamples, room, l, 'Test Samples');
%testSamples = TraditionalTestSamples();
%PlotSamples(testSamples, room, l, 'Test Samples');
count = size(testSamples,1);

% load best fit parameters and rebuild the fuzzy controllers
fis1 = readfis('robotpos_k');
fis2 = readfis('robotori_k');
%params = load('../TrainedGenes/model_k6_r500.mat');
params = load('BestGenes/Gen0.mat');
[fis1, fis2] = LoadFuzzyControllers_k(params.R, fis1, fis2);

failure = 0;
maxStep = 60;
for i = 1:count
    [cost, step, success, trajectory] = Trajectory(testSamples(i,:), room, l, fis1, fis2, maxStep);
    PlotTrajectory(trajectory, room, l, 0.1, true);
    fprintf('%d th: success: %d \t total distance: %.3f \t step: %d \n',i, success, cost, step);
    if ~success
        failure = failure + 1;
    end
end
fprintf('%d failure / %d total \n', failure, count);

function [totalDist, step, success, trajectory] = Trajectory(sample, room, L, fisPos, fisOri, maxStep)
totalDist = 0;
r1 = sample(1:2);
r2 = sample(3:4);
rodAngle = sample(5);
targetY = room(4)+room(6);
trajectory = [r1 r2 false];
success = true;
step = 0;
while r1(2) <= targetY || r2(2) <= targetY
    if step > maxStep
        success = false;
        break;
    end
    
    [dist, r1, r2, rodAngle, hit] = RobotsMoveWithControl(r1, r2, rodAngle, fisPos, fisOri, L, room);
    trajectory = [trajectory; r1 r2 hit];
    if hit
        success = false;
    end
    
    totalDist = totalDist+dist;
    step = step+1;
end
end
