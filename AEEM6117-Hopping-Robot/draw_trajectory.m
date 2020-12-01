function draw_trajectory()
load('data.mat')
% t = ans.time;
data = ans.data;
x = data(:,1);
y = data(:,2);
% f = data(:,3);
% p = polyfit(x,y,5);
% x1 = linspace(0,29);
% y1 = polyval(p,x1);
set(gcf,'Position',[100,100,500,500])
axis equal

hold on
axis([0,30,0,15]);
figure(1)
plot(x,y,'k--');
title('jumping performance (inclination angle = 45 degree)');
xlabel('displacement (M)');
ylabel('height (M)');
% plot(x1,y1,'r--');
% slope angle = 0
plot_slope(30,10,'b');
% plot([24.22,24.22],[0,4.297],'b');
plot_slope(30,20,'r');
% plot([18.75,18.75],[0,6.845],'r');
plot_slope(30,30,'k');
% plot([12.5,12.5],[0,7.246],'k');
plot_slope(30,40,'g');
% plot([4.697,4.697],[0,3.981],'g');

legend('jumping trajectory','slope (angle = 10 degree)', 'slope (angle = 20 degree)', 'slope (angle = 30 degree)', 'slope (angle = 40 degree)')

hold off
end

function plot_slope(x,theta,color)
plot([0,x],[0,x*tan(theta*pi/180)],color);
end