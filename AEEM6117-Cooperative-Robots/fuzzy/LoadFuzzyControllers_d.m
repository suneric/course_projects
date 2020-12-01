% load robot fuzzy controllers
% the parameters fuzzy controllers will be with the input parameters vector
function [fis1, fis2] = LoadFuzzyControllers_d(params, fis1, fis2)
% for each fuzzy controller
% there are 12 parameters in float value for member functions
% there are 50 parameters for rules
p = params(1:12);
r = round(params(13:62));

% robot1
fis1.input(1).mf(1).params = [-1 -1 -p(1)];
fis1.input(1).mf(2).params = [-1 -p(1) 0];
fis1.input(1).mf(3).params = [-p(1) 0 p(2)];
fis1.input(1).mf(4).params = [0 p(2) 1];
fis1.input(1).mf(5).params = [p(2) 1 1];
fis1.input(2).mf(1).params = [-1 -1 -p(3)];
fis1.input(2).mf(2).params = [-1 -p(3) 0];
fis1.input(2).mf(3).params = [-p(3) 0 p(4)];
fis1.input(2).mf(4).params = [0 p(4) 1];
fis1.input(2).mf(5).params = [p(4) 1 1];
fis1.output(1).mf(1).params = [-1 -1 -p(5)];
fis1.output(1).mf(2).params = [-1 -p(5) 0];
fis1.output(1).mf(3).params = [-p(5) 0 p(6)];
fis1.output(1).mf(4).params = [0 p(6) 1];
fis1.output(1).mf(5).params = [p(6) 1 1];
for i = 1:25
    fis1.rule(i).consequent = r(i);
end

% robot2
fis2.input(1).mf(1).params = [-1 -1 -p(7)];
fis2.input(1).mf(2).params = [-1 -p(7) 0];
fis2.input(1).mf(3).params = [-p(7) 0 p(8)];
fis2.input(1).mf(4).params = [0 p(8) 1];
fis2.input(1).mf(5).params = [p(8) 1 1];
fis2.input(2).mf(1).params = [-1 -1 -p(9)];
fis2.input(2).mf(2).params = [-1 -p(9) 0];
fis2.input(2).mf(3).params = [-p(9) 0 p(10)];
fis2.input(2).mf(4).params = [0 p(10) 1];
fis2.input(2).mf(5).params = [p(10) 1 1];
fis2.output(1).mf(1).params = [-1 -1 -p(11)];
fis2.output(1).mf(2).params = [-1 -p(11) 0];
fis2.output(1).mf(3).params = [-p(11) 0 p(12)];
fis2.output(1).mf(4).params = [0 p(12) 1];
fis2.output(1).mf(5).params = [p(12) 1 1];
for i = 1:25
    fis1.rule(i).consequent = r(i+25);
end
end