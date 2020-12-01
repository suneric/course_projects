function out=Sensor(in,P)
pn=in(1);
pe=in(2);
pd=in(3);
phi=in(7)+randn(1)*0.1;
theta=in(8)+randn(1)*0.1;
p=in(10)+randn(1)*0.1;
q=in(11)+randn(1)*0.1;
r=in(12)+randn(1)*0.1;
T=P.m*P.g;
rho=zeros(P.N+1,1);
for i=1:P.N
    %land mark position
    pn1=P.Xf(1,i);
    pe1=P.Xf(2,i);
    pd1=0;
    % range sensor 
    rho(i)=((pn-pn1)^2+(pe-pe1)^2+(pd-pd1)^2)^0.5+randn(1)*0.1;    
end
out=[phi;theta;p;q;r;T;rho];
end