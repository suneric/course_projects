% objective function
%
%
function cost = CostFunc_d(params, samples, room, M, L, fis, fis1, fis2, deltaT,maxStep)
[fis1, fis2] = LoadFuzzyControllers_d(params, fis1, fis2);
cost = 0;
for i = 1:size(samples,1)
    [c, ~] = CreateTrajectory_d(samples(i,:), room, M, L, fis, fis1, fis2, deltaT,maxStep);
    cost = cost + c;
end
end 


