% validation function after each generation
%
function [state,options,optchanged] = ValidationFunc(options,state, flag)
optchanged = false;
% find the best fit population in the generation
idx = find(state.Score == min(state.Score),1);
R = state.Population(idx,:);
save([pwd sprintf('/BestGenes/Gen%d.mat', state.Generation)],'R');

if state.Generation==0
    fprintf('Generation \t \t Best Cost \t \t Mean Cost \n');
elseif state.Generation>=1
    fprintf('\t %d \t \t %.2f \t \t %.2f \n',state.Generation, min(state.Score), mean(state.Score));
end
end