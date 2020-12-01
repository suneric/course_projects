% objective function
%
%
function cost = CostFunc_k(params, samples, room, L, fis1, fis2, maxStep)
[fis1, fis2] = LoadFuzzyControllers_k(params, fis1, fis2);
cost = 0;
for i = 1:size(samples,1)
    [c, ~] = CreateTrajectory_k(samples(i,:), room, L, fis1, fis2, maxStep);
    cost = cost + c;
end
end 


