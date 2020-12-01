clear
clc % clean command window

warning('off','all')
rng('shuffle');

% define room
% [roomx_min, roomx_max, roomy_min, roomy_max, door_width, door_dpeth]
room = [-1 1 -1 0 0.2 0.2];

testSamples = RandomRobotPosition(room, 10);
count = size(testSamples,1);

fis = readfis('robot');
params = load('./BestGenes/Gen100.mat');
[fis] = LoadFuzzyController(params.R, fis);

failure = 0;
maxStep = 250;
deltaT = 0.05;
for i = 1:count
    [cost, step, success, trajectory] = CreateTrajectory(testSamples(i,:), room, fis, maxStep, deltaT);
    PlotRobotTrajectory(trajectory, room, 0.1, true);
    fprintf('%d th: success: %d \t total distance: %.3f \t step: %d \n',i, success, cost, step);
    if ~success
        failure = failure + 1;
    end
end
fprintf('%d failure / %d total \n', failure, count);

function PlotRobotTrajectory(trajectory, room, deltaT, clear)
figure('Name', 'Robot Trajectory');
subplot(1,1,1);
count = size(trajectory,1); % get row count
for i = 1:count
    if clear
        clf;
    end
    hold on
    % plot room
    roomx = linspace(room(1),room(2),100);
    roomy = linspace(room(3),room(4)+room(6)+0.5,100);
    plot(roomx, 0, 'k');
    plot(0, roomy, 'k');

    % plot door
    plot([room(1) -room(5)/2], [room(4) room(4)], 'k');
    plot([room(1) -room(5)/2], [room(4)+room(6) room(4)+room(6)], 'k');
    plot([room(5)/2, room(2)], [room(4) room(4)], 'k');
    plot([room(5)/2, room(2)], [room(4)+room(6) room(4)+room(6)], 'k');
    plot([-room(5)/2, -room(5)/2], [room(4) room(4)+room(6)], 'k');
    plot([room(5)/2, room(5)/2], [room(4) room(4)+room(6)], 'k');
    plot(trajectory(i,1),trajectory(i,2),'bo');
    pause(deltaT);
end
    plot(trajectory(:,1),trajectory(:,2),'bo');
end