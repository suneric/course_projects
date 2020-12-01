function out = force(in,P)
out(1)=in(1);
out(2)=in(2);
y = in(2);
if (y >= P.l*cos(P.th))
    out(3) = 0;
else
    out(3) = P.k*(P.l-y/cos(P.th));
end