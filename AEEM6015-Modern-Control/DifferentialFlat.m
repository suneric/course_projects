function out=DifferentialFlat(in,P)
% desired trajectory with flat output
pn_d = in(1);
pe_d = in(2);
pd_d = in(3);
psi_d = in(4);
pnd_d = in(5);
ped_d = in(6);
pdd_d = in(7);
psid_d = in(8);
pndd_d = in(9);
pedd_d = in(10);
pddd_d = in(11);
% linearize the system with ur
% desired u_r =[up_r;upsi_r]; 
up_r = [pndd_d;pedd_d;pddd_d]-[0;0;P.g];
upsi_r = psid_d;
u_r = [up_r;upsi_r]; 
x_r = [pn_d;pe_d;pd_d;pnd_d;ped_d;pdd_d;psi_d];
out=[u_r;x_r];
end
