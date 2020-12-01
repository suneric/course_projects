function print_data()
load('data.mat')
t = ans.time;
data = ans.data;
x = data(:,1);
y = data(:,2);
f = data(:,3);

hold on
figure(1)
h1 = subplot(3,1,1);
plot(t,f);
title(h1,'spring force (N) to time (s)');
xlabel(h1, 'time (s)');
ylabel(h1, 'force (N)');
h2 = subplot(3,1,2);
plot(t,y);
title(h2,'height (m) to time (s)');
xlabel(h2, 'time (s)');
ylabel(h2, 'height (m)');
h3 = subplot(3,1,3);
plot(t,x);
title(h3,'distance (m) to time (s)');
xlabel(h3, 'time (s)');
ylabel(h3, 'distance (m)');
hold off

end