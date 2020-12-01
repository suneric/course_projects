% load robot fuzzy controllers
% the parameters fuzzy controllers will be with the input parameters vector
function [fispos, fisori] = LoadFuzzyControllers_k(params, fispos, fisori)
% reset fis with the parameters
p = params(1:18); % params
r = round(params(19:153)); % rules
fispos.input(1).mf(1).params = [-1 -1 -p(1) 0];
fispos.input(1).mf(2).params = [-p(1) 0 p(2)]; 
fispos.input(1).mf(3).params = [0 p(2) 1 1]; 
fispos.input(2).mf(1).params = [-1 -1 -p(3) 0]; 
fispos.input(2).mf(2).params = [-p(3) 0 p(4)]; 
fispos.input(2).mf(3).params = [0 p(4) 1 1]; 
fispos.input(3).mf(1).params = [-1 -1 -p(5)]; 
fispos.input(3).mf(2).params = [-1 -p(5) 0]; 
fispos.input(3).mf(3).params = [-p(5) 0 p(6)]; 
fispos.input(3).mf(4).params = [0 p(6) 1]; 
fispos.input(3).mf(5).params = [p(6) 1 1];
fispos.output(1).mf(1).params = [-1 -1 -p(7) 0];
fispos.output(1).mf(2).params = [-p(7) 0 p(8)];
fispos.output(1).mf(3).params = [0 p(8) 1 1];
fispos.output(2).mf(1).params = [-1 -1 -p(9) 0];
fispos.output(2).mf(2).params = [-p(9) 0 p(10)];
fispos.output(2).mf(3).params = [0 p(10) 1 1];
for i = 1:45
    fispos.rule(i).consequent = [r(2*i-1),r(2*i)];
end
fisori.input(1).mf(1).params = [-1 -1 -p(11) 0]; 
fisori.input(1).mf(2).params = [-p(11) 0 p(12)]; 
fisori.input(1).mf(3).params = [0 p(12) 1 1]; 
fisori.input(2).mf(1).params = [-1 -1 -p(13) 0];
fisori.input(2).mf(2).params = [-p(13) 0 p(14)]; 
fisori.input(2).mf(3).params = [0 p(14) 1 1]; 
fisori.input(3).mf(1).params = [-1 -1 -p(15)]; 
fisori.input(3).mf(2).params = [-1 -p(15) 0]; 
fisori.input(3).mf(3).params = [-p(15) 0 p(16)]; 
fisori.input(3).mf(4).params = [0 p(16) 1]; 
fisori.input(3).mf(5).params = [p(16) 1 1];
fisori.output(1).mf(1).params = [-1 -1 -p(17) 0];
fisori.output(1).mf(2).params = [-p(17) 0 p(18)];
fisori.output(1).mf(3).params = [0 p(18) 1 1];
for i = 1:45
    fisori.rule(i).consequent = r(90+i);
end
end