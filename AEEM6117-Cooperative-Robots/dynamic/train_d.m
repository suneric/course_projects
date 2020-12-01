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

% predefine the mass and length of the rod
m = 1;
l = 0.5;

%trainSamples = CreateRandomSamples(l, room, 50);
trainSamples = RandomTrainingSamples();

fis1 = readfis('robot_d');
fis2 = readfis('robot_d');

fis = readfis('robot');
singleFisParams = load('model_single_r100.mat');
fis = LoadFuzzyController(singleFisParams.R, fis);

deltaT = 0.01;
maxStep = 500;
outputFunc = @(options,state,flag) ValidationFunc(options,state,flag);
fitnessFunc = @ (vec) CostFunc_d(vec,trainSamples,room,m,l,fis,fis1,fis2,deltaT,maxStep);
options = optimoptions('ga',...
                       'Generations',100,...
                       'PopulationSize',100,...
                       'TolFun',1e-6,...
                       'OutputFcn',outputFunc,...
                       'UseParallel',true,...
                       'PlotFcn', 'gaplotbestf');

% there are 510 variables need to be tuned by GA  
nvar = 62;
range = [zeros(1,12) 0.6*ones(1,50); ones(1,12) 5.4*ones(1,50)];
[pop, fit] = ga(fitnessFunc,nvar,[],[],[],[],range(1,:),range(2,:),[],[],options); 