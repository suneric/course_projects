function out=PlotTrajectory(in)
t = in(1);
traj_d = in(2:4);
traj = in(14:20);
pn = traj(1);
pe = traj(2);
pd = traj(3);
pnd = traj_d(1);
ped = traj_d(2);
pdd = traj_d(3);

persistent traj_handle;
persistent trajd_handle;
persistent x;
persistent xd;
% first time function is called, initialize plot and persistent vars
if t<=0.1,
    x=[];
    xd=[];
    figure(2), clf
    %draws quadrotor at initial position
    hold on
    trajd_handle=plot3(pn,pe,pd,'r','linewidth',1.5,'linestyle','--','erasemode','normal');
    traj_handle=plot3(pn,pe,pd,'b','linewidth',1.5,'linestyle','--','erasemode','normal');
    axis([-3,3,-3,3,0,2]); % see if it is possible to change the axis and not distort the image
    xlabel('north')
    ylabel('east')
    zlabel('down')
    grid on
    view(-44,22)  % set the vieew angle for figure  % set the view angle for figure
    % at every other time step, redraw base and rod
else
    x=[x [pn;pe;pd]];
    xd =[xd [pnd;ped;pdd]];
    set(traj_handle, 'xdata', x(1,:), 'ydata', x(2,:),'zdata',-x(3,:));
    set(trajd_handle, 'xdata', xd(1,:), 'ydata', xd(2,:),'zdata',-xd(3,:));
    drawnow limitrate % ;
end

end