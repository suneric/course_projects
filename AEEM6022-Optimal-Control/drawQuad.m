%==========================================================================
%This is the quadrotor drawing code given to ECE 6320 Students
%This is used attitide control and position control ()
%
%09/25/2014: Last modified by Rajikant Sharma
%===============================================================================

function drawQuad(uu,P)

% process inputs to function
pn       = uu(1);       % inertial North position
pe       = uu(2);       % inertial East position
pd       = uu(3);
u        = uu(7);
v        = uu(8);
w        = uu(9);
phi      = uu(4);       % roll angle
theta    = uu(5);       % pitch angle
psi      = uu(6);       % yaw angle
p        = uu(10);       % roll rate
q        = uu(11);       % pitch rate
r        = uu(12);       % yaw rate
t        = uu(13); % time

%rotate phi and theta according to psi
R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
tmp = R * [phi;theta];
phi = tmp(1);
theta = tmp(2);

% define persistent variables
persistent fig_quadrotor;
persistent traj_handle;
persistent x;

% first time function is called, initialize plot and persistent vars
if t<=0.1,
    figure(1), clf
    x=[];
    %draws quadrotor at initial position
    fig_quadrotor = drawquadrotor(pn,pe,pd,phi,theta,psi, P, [], 'normal');
    hold on
    traj_handle=plot3(pn,pe,pd,'b','linewidth',1.5,'linestyle','--','erasemode','normal');
    axis([-3,3,-3,3,0,6]); % see if it is possible to change the axis and not distort the image
    xlabel('north')
    ylabel('east')
    zlabel('down')
    grid on
    view(-44,22)  % set the vieew angle for figure  % set the view angle for figure
    % at every other time step, redraw base and rod
else
    x=[x [pn;pe;pd]];
    drawquadrotor(pn,pe,pd,phi,theta,psi, P, fig_quadrotor);
    set(traj_handle, 'xdata', x(1,:), 'ydata', x(2,:),'zdata',-x(3,:));
    drawnow limitrate % ;
end
end


%=======================================================================
% drawquadrotor
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
function handle = drawquadrotor(pn,pe,pd,phi,theta,psi, P, handle, mode)

% define points on spacecraft
[V, F, patchcolors] = quadrotorVFC(P);

% rotate spacecraft
V = rotateVert(V, phi, theta, psi);

% translate spacecraft
V = translateVert(V, [pn; pe; -pd]);

if isempty(handle),
    handle = patch('Vertices', V, 'Faces', F,...
        'FaceVertexCData',patchcolors,...
        'FaceColor','flat',...
        'EraseMode', mode);
else
    set(handle,'Vertices',V,'Faces',F);
    drawnow limitrate % eric note: 'limitrate' make your simulate faster.
end
end

%=======================================================================
% drawpillarsl / draws left pillar
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================

function [V, F, patchcolors]=quadrotorVFC(P)

% define the robot center
rotorcenter = [...
    -P.boxlength-P.armlength P.ystart P.zstart+P.boxheight-P.armposition+P.armwidth;... %54 left rotor center
    P.boxlength+P.armlength P.ystart P.zstart+P.boxheight-P.armposition+P.armwidth;...%55 right rotor center
    0 P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;...%56 front rotor center
    0 P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;...%57 back rotor center 
    ];

% Define the vertices (physical location of vertices)
V = [...
    -P.boxlength P.boxwidth 0;... %1  center box
    -P.boxlength -P.boxwidth 0;... %2
    -P.boxlength -P.boxwidth P.boxheight;... %3
    -P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %4
    P.boxlength P.ystart+P.boxwidth P.zstart;... %5
    P.boxlength P.ystart-P.boxwidth P.zstart;... %6
    P.boxlength P.ystart-P.boxwidth P.zstart+P.boxheight;... %7
    P.boxlength P.ystart+P.boxwidth P.zstart+P.boxheight;... %8
    -P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %9 arm left
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %10
    -P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %11
    -P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %12
    -P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %13
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %14
    -P.boxlength-P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %15
    -P.boxlength-P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %16
    P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %17 arm right
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %18
    P.boxlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %19
    P.boxlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %20
    P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %21
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition-P.armwidth;... %22
    P.boxlength+P.armlength P.ystart-P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %23
    P.boxlength+P.armlength P.ystart+P.armwidth P.zstart+P.boxheight-P.armposition+P.armwidth;... %24
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %25 arm front
    P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %26
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %27
    -P.armwidth P.ystart+P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %28
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %29
    P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %30
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %31
    -P.armwidth P.ystart+P.boxlength+P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %32
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %33 back front
    P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %34
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %35
    -P.armwidth P.ystart-P.boxlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %36
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %37
    P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %38
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition+P.armwidth;... %39
    -P.armwidth P.ystart-P.boxlength-P.armlength P.zstart+P.boxheight-P.armposition-P.armwidth;... %40
    0 P.ystart-P.perimeter P.zstart+P.boxheight;... %41 perimeter
    0 P.ystart-P.perimeter P.zstart+P.boxheight-P.periwidth;... %42
    0 P.ystart+P.perimeter P.zstart+P.boxheight;... %43
    0 P.ystart+P.perimeter P.zstart+P.boxheight-P.periwidth;... %44
    P.perimeter P.ystart P.zstart+P.boxheight;... %45
    P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %46
    -P.perimeter P.ystart P.zstart+P.boxheight;... %47
    -P.perimeter P.ystart P.zstart+P.boxheight-P.periwidth;... %48
    0 0 P.boxheight+(1/12);... %49 camera
    -1/24 1/12 P.boxheight+(1/12)-(1/32);... %50
    1/24 1/12 P.boxheight+(1/12)-(1/32);... %51
    -1/24 1/12 P.boxheight+(1/12)+(1/32);... %52
    1/24 1/12 P.boxheight+(1/12)+(1/32);... %53
    rotorcenter(1,:);... %54 left rotor center;
    rotorcenter(1,1)-0.3*cos(pi/4) rotorcenter(1,2)+0.3*cos(pi/4) rotorcenter(1,3);... % 55
    rotorcenter(1,1) rotorcenter(1,2)+0.3 rotorcenter(1,3);... %56
    rotorcenter(1,1)+0.3*cos(pi/4) rotorcenter(1,2)+0.3*cos(pi/4) rotorcenter(1,3);... % 57
    rotorcenter(1,1)+0.3 rotorcenter(1,2) rotorcenter(1,3);... %58
    rotorcenter(1,1) rotorcenter(1,2)-0.3 rotorcenter(1,3);... % 59
    rotorcenter(1,1)-0.3*cos(pi/4) rotorcenter(1,2)-0.3*cos(pi/4) rotorcenter(1,3);... %60
    rotorcenter(2,:);... %61 right rotor center
    rotorcenter(2,1)-0.3*cos(pi/4) rotorcenter(2,2)+0.3*cos(pi/4) rotorcenter(2,3);... % 62
    rotorcenter(2,1) rotorcenter(2,2)+0.3 rotorcenter(1,3);... %63
    rotorcenter(2,1)+0.3*cos(pi/4) rotorcenter(2,2)+0.3*cos(pi/4) rotorcenter(2,3);... % 64
    rotorcenter(2,1)+0.3 rotorcenter(2,2) rotorcenter(2,3);... %65
    rotorcenter(2,1) rotorcenter(2,2)-0.3 rotorcenter(2,3);... % 66
    rotorcenter(2,1)-0.3*cos(pi/4) rotorcenter(2,2)-0.3*cos(pi/4) rotorcenter(2,3);... %67
    rotorcenter(3,:);... %54 front rotor center
    rotorcenter(3,1)-0.3*cos(pi/4) rotorcenter(3,2)+0.3*cos(pi/4) rotorcenter(3,3);... % 62
    rotorcenter(3,1) rotorcenter(3,2)+0.3 rotorcenter(3,3);... %63
    rotorcenter(3,1)+0.3*cos(pi/4) rotorcenter(3,2)+0.3*cos(pi/4) rotorcenter(3,3);... % 64
    rotorcenter(3,1)+0.3 rotorcenter(3,2) rotorcenter(3,3);... %65
    rotorcenter(3,1) rotorcenter(3,2)-0.3 rotorcenter(3,3);... % 66
    rotorcenter(3,1)-0.3*cos(pi/4) rotorcenter(3,2)-0.3*cos(pi/4) rotorcenter(3,3);... %67
    rotorcenter(4,:);... %54 back rotor center
    rotorcenter(4,1)-0.3*cos(pi/4) rotorcenter(4,2)+0.3*cos(pi/4) rotorcenter(4,3);... % 62
    rotorcenter(4,1) rotorcenter(4,2)+0.3 rotorcenter(4,3);... %63
    rotorcenter(4,1)+0.3*cos(pi/4) rotorcenter(4,2)+0.3*cos(pi/4) rotorcenter(4,3);... % 64
    rotorcenter(4,1)+0.3 rotorcenter(4,2) rotorcenter(4,3);... %65
    rotorcenter(4,1) rotorcenter(4,2)-0.3 rotorcenter(4,3);... % 66
    rotorcenter(4,1)-0.3*cos(pi/4) rotorcenter(4,2)-0.3*cos(pi/4) rotorcenter(4,3);... %67
    ]*0.3;
% %V=[Vtemp(:,1:2) -Vtemp(:,3)];
% psi=-pi/2;
%  R = [...
%           cos(psi), sin(psi), 0;...
%           -sin(psi), cos(psi), 0;...
%           0, 0, 1];
%         R = R';
% 
%   % rotate vertices
%   V= (R*V')';

% define faces as a list of vertices numbered above
F = [...
    1, 2, 3, 4;... % left
    5, 6, 7, 8;... % right
    2, 6, 7, 3;... % back
    1, 5, 8, 4;... % front
    4, 3, 7, 8;... % top
    1, 2, 6, 5;... % bottom
    16, 15, 11, 12;... %left arm
    15, 14, 10, 11;...
    13, 14, 10, 9;...
    13, 9, 12, 16;...
    19, 23, 22, 18;... %right arm
    20, 19, 23, 24;...
    17, 18, 22, 21;...
    17, 21, 24, 20;...
    31, 27, 28, 32;... %front arm
    29, 32, 28, 25;...
    29, 25, 26, 30;...
    31, 27, 26, 30;...
    35, 39, 38, 34;... %back arm
    35, 36, 40, 39;...
    33, 36, 40, 37;...
    34, 37, 38, 34;...
    41, 42, 46, 45;... %perimeter
    45, 46, 44, 43;...
    43, 44, 48, 47;...
    47, 48, 42, 41;...
    49, 50, 51, 49;... %camera
    49, 50, 52, 49;...
    49, 52, 53, 49;...
    49, 51, 53, 49;...
    54, 54, 55, 56;... % left rotor
    54, 54, 57, 58;...
    54, 54, 59, 60;...
    61, 61, 62, 63;... % right rotor
    61, 61, 64, 65;...
    61, 61, 66, 67;...
    68, 68, 69, 70;... % front rotor
    68, 68, 71, 72;...
    68, 68, 73, 74;...
    75, 75, 76, 77;... % back rotor
    75, 75, 78, 79;...
    75, 75, 80, 81;...
    ];

% define colors for each face
myred = [1, 0, 0];
mygreen = [0, 1, 0];
myblue = [0, 0, 1];
myyellow = [1, 1, 0];
mycyan = [0, 1, 1];
myrotorclr=[0.5,1,0.5];

patchcolors = [...
    mygreen;... % left
    mygreen;... % right
    mygreen;... % back
    myred;... % front
    myred;... % top
    mycyan;... % bottom
    myblue;... %left arm
    myblue;...
    myblue;...
    myblue;...
    myblue;... %right arm
    myblue;...
    myblue;...
    myblue;...
    myyellow;... %front arm
    myyellow;...
    myyellow;...
    myyellow;...
    myblue;... %back arm
    myblue;...
    myblue;...
    myblue;...
    myred;... %perimeter
    myred;...
    myred;...
    myred;...
    myyellow;...%camera
    myyellow;...
    myyellow;...
    myyellow;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    myrotorclr;...
    ];
end


%%%%%%%%%%%%%%%%%%%%%%%not done
function Vert=rotateVert(Vert,phi,theta,psi)
% 
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  Vert= (R*Vert')';


end % rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by column vector T
%not done
function Vert = translateVert(Vert, T)

Vert = Vert + repmat(T', size(Vert,1),1);

end % translateVert