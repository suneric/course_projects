function [fis] = LoadFuzzyController(params, fis)
p = params(1:10);
r = round(params(11:91));
fis.input(1).mf(1).params = [-1 -1 -p(1) 0];
fis.input(1).mf(2).params = [-p(1) 0 p(2)]; 
fis.input(1).mf(3).params = [0 p(2) 1 1]; 
fis.input(2).mf(1).params = [-1 -1 -p(3) 0]; 
fis.input(2).mf(2).params = [-p(3) 0 p(4)]; 
fis.input(2).mf(3).params = [0 p(4) 1 1];
fis.input(3).mf(1).params = [-10 -10 -p(5) 0];
fis.input(3).mf(2).params = [-p(5) 0 p(6)]; 
fis.input(3).mf(3).params = [0 p(6) 10 10]; 
fis.input(4).mf(1).params = [-10 -10 -p(7) 0]; 
fis.input(4).mf(2).params = [-p(7) 0 p(8)]; 
fis.input(4).mf(3).params = [0 p(8) 10 10];
fis.output(1).mf(1).params = [-1 -1 -p(9)];
fis.output(1).mf(2).params = [-1 -p(9) 0];
fis.output(1).mf(3).params = [-p(9) 0 p(10)];
fis.output(1).mf(4).params = [0 p(10) 1];
fis.output(1).mf(5).params = [p(10) 1 1];
for i = 1:81
    fis.rule(i).consequent = r(i);
end
end