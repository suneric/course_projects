% This is a simulation of single robot moving out of a room
%

clear
clc % clean command window

warning('off','all');
rng('shuffle'); 

% define room
% [roomx_min, roomx_max, roomy_min, roomy_max, door_width, door_dpeth]
room = [-1 1 -1 0 0.2 0.2];
trainSamples = RandomRobotPosition(room, 50);
%PlotRobotSample(trainSamples, room);

fis = readfis('robot');
deltaT = 0.1;
maxStep = 150;
outputFunc = @(options,state,flag) ValidationFunc(options,state,flag);
fitnessFunc = @ (vec) CostFunc(vec,trainSamples,room,fis,maxStep,deltaT);
options = optimoptions('ga',...
                       'Generations',100,...
                       'PopulationSize',200,...
                       'TolFun',1e-6,...
                       'OutputFcn',outputFunc,...
                       'UseParallel',true,...
                       'PlotFcn', 'gaplotbestf');

nvar = 91;
range = [zeros(1,10) 0.6*ones(1,81); ones(1,4) 10*ones(1,4) ones(1,2) 5.4*ones(1,81)];
[pop, fit] = ga(fitnessFunc,nvar,[],[],[],[],range(1,:),range(2,:),[],[],options); 

function cost = CostFunc(params, samples, room, fis, maxStep, deltaT)
[fis] = LoadFuzzyController(params, fis);
cost = 0;
for i = 1:size(samples,1)
    [c, ~] = CreateTrajectory(samples(i,:), room, fis, maxStep, deltaT);
    cost = cost + c;
end
end 

function PlotRobotSample(samples, room)
figure('Name', 'Robot Sample');
subplot(1,1,1);
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
    plot(samples(:,1),samples(:,2),'bo');
end