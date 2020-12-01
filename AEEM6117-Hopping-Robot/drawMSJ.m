function drawMSJ(in,P)
x = in(1);
y = in(2);
figure(1), clf
hold on
title('mass-spring-jump')
% plot a circle for mass
th=0:pi/50:2*pi;
r = 0.5;
xunit = r*cos(th)+x;
yunit = r*sin(th)+y+r;
plot(xunit,yunit,'k-');
if (y > P.l*cos(P.th))
    plot([x x-P.l*sin(P.th)],[y,y-P.l*cos(P.th)],'b-');
elseif (y < P.l)
    plot([x x-y*tan(P.th)],[y,0],'b-');
end
plot([-1 1],[0 0],'b');
xlabel('x (meter)');
ylabel('y (meter)');
axis([-1 30 0 30]);

th2 = P.th2 % angle of slope
plot([0,30],[0,30*tan(th2)])

hold off
end