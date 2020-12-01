% This is a simulation of 2 robots carry a rod through an opening
% with considering dynamics 
%

clear
clc % clean command window

warning('off','all');
rng('shuffle'); 

% define room
% [roomx_min, roomx_max, roomy_min, roomy_max, door_width, door_dpeth]
room = [-1 1 -1 0 0.2 0.2];

% predefine the length of the rod
l = 0.5;

%trainSamples = CreateTrainingSamples(l); 
trainSamples = RandomTrainingSamples();
PlotSamples(trainSamples, room, l, 'Training Samples');

fis1 = readfis('robotpos_k');
fis2 = readfis('robotori_k');

maxStep = 50;
outputFunc = @(options,state,flag) ValidationFunc(options,state,flag);
fitnessFunc = @ (vec) CostFunc_k(vec,trainSamples,room,l,fis1,fis2,maxStep);
options = optimoptions('ga',...
                       'Generations',500,...
                       'PopulationSize',100,...
                       'TolFun',1e-12,...
                       'OutputFcn',outputFunc,...
                       'UseParallel',true,...
                       'PlotFcn', 'gaplotbestf');

% there are 153 variables need to be tuned by GA  
% including 18 parameters for member functions and 135 for infrence rules
nvar = 153;
range = [zeros(1,18) 0.6*ones(1,135); ones(1,18) 3.4*ones(1,135)];

tStart = tic;
[pop, fit] = ga(fitnessFunc,nvar,[],[],[],[],range(1,:),range(2,:),[],[],options); 
tElapse = toc(tStart);
fprintf('time: %d hr', tElapse/3600);