function out=AttitudeTrajectory(in,P)
t = in;
out = Attitude(t,P);
end

function out=Attitude(t,P)
phi = 0.01;
theta = 0;
psi_r = 0.2;
F = P.m*P.g;
out = [F;phi;theta;psi_r];
end