function PlotTraj()
load('traj.mat');
t=Xt.time;
data=Xt.data;
pn_d=data(:,1);
pe_d=data(:,2);
pd_d=data(:,3);
pn = data(:,8);
pe = data(:,9);
pd = data(:,10);
hold on
figure(2)
h1 = subplot(3,1,1);
plot(t,(pn-pn_d));
title(h1,'pn err')
h2 = subplot(3,1,2);
plot(t,(pe-pe_d));
title(h2,'pe err')
h3 = subplot(3,1,3);
plot(t,(pd-pd_d));
title(h3,'pd err')
hold off
end