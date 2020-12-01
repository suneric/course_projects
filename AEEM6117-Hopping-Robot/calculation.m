% K=24000*9.81 N/m 
% mass = 100 kg

g = 9.81/6; %N/kg
v0 = 6.7637; % m/s

% data = [];
% for i=[0 10 20 30]
%     for j=[45]%[5 10 15 20 25 30 35 40 45 50 55 60 65 70 75 80 85]
%        theta = (i/180)*pi;
%        beta = (j/180)*pi;
%        [x,h] = JumpDistanceOnSlope(theta,beta,v0,g);
%        s = x/cos(theta);
%        data = [data,[i;j;s;h]];
%     end
% end

hold on
figure(1)
jump_curve = Parabola((45/180)*pi,v0,g)
plot(jump_curve[:;1],jump_curve[:;2]);

function curve = Parabola(beta,v0,g)
h = 0.5*v0^2*cos(beta)^2/g;
b = 2*tan(beta)*h
data = [];
for x = 0:0.5:2*b
    y = -(x-b)^2+h;
    data = [data,[x;y]];
end
curve = data;
end

function [x,h]=JumpDistanceOnSlope(theta,beta,v0,g)
h = 0.5*v0^2*cos(beta)^2/g;
b = 2*tan(beta)*h;
x = 0.5*(2*b-tan(theta)+sqrt((tan(theta)-2*b)^2-4*(b^2-h)));
end