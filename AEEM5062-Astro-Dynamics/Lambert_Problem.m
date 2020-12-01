% find the orbit energy
function out=Lambert_Problem()
u = 398600;%km^3/s^2
t = 1800; %s
r1_vec = [3600 4600 3600];
r2_vec = [-5500 6240 -5200];

% Step 1: calculate r1 and r2
r1 = norm(r1_vec);
r2 = norm(r2_vec);

% Step 2:calculate A
% choose prograde trajectory for calculating delta theta
temp = cross(r1_vec, r2_vec);
if (temp(3) > 0)
    dTheta = acos(dot(r1_vec,r2_vec)/(r1*r2));
else
    dTheta = 2*pi-acos(dot(r1_vec,r2_vec)/(r1*r2));
end
A = sin(dTheta)*sqrt(r1*r2/(1-cos(dTheta)));

% Step 3:find z value making F = 0
F = @(z) Newton_Law(z,r1,r2,A,u,t);
z = fzero(F,[0 5]);

% Step 4: evaluate S and C
%[S,C] = Stumpff(z);

% Step 5: evaluate Y
y = Y(z,r1,r2,A);

% Step 6: evaluate lagrange coeefficient
f = 1-y/r1;
g = A*sqrt(y/u);

% Step 7: evaluate velocity 
v1_vec = (1/g)*(r2_vec-f*r1_vec);
v1 = norm(v1_vec);

% Step 8: evaluate energu
E = 0.5*v1^2-u/r1;
out = E;
end

function out=Newton_Law(z,r1,r2,A,u,t)
[S,C] = Stumpff(z);
y = Y(z,r1,r2,A);
out = ((y/C)^1.5)*S+A*sqrt(y)-sqrt(u)*t;
end

function out = Y(z,r1,r2,A)
[S,C] = Stumpff(z);
out = r1+r2+A*((z*S-1)/sqrt(C));
end

function [S,C]=Stumpff(z)
if (z > 0)
    S = (sqrt(z)-sin(sqrt(z)))/sqrt(z)^3;
    C = (1-cos(sqrt(z)))/z;
elseif (z < 0)
    S = (sinh(sqrt(-z))-sqrt(-z))/sqrt(z)^3;
    C = (cos(sqrt(-z))-1)/(-z);
else
    S = 1/6;
    C = 0.5;
end
end