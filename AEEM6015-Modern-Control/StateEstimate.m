function out=StateEstimate(in)
pn=in(1);
pe=in(2);
pd=in(3);
vn=in(4);
ve=in(5);
vd=in(6);
psi=in(9);
out=[pn;pe;pd;vn;ve;vd;psi];
end