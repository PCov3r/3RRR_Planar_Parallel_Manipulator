%% This program implements the kinetostatics of our robot using the Virtual Work Principle. 
%% It outputs the torques necessary to achieve a desired path.

clear all

% working mode = +++ = 111
wm1 = 1;
wm2 = 1;
wm3 = 1;

% resting position
xini = 0;
yini = 0;
alphaini = 0;

% robot's path, here: a circle of radius 10 centred in (0,0)
ang=linspace(0,2*pi,36);
xp=10*cos(ang);
yp=10*sin(ang);
x = 0;
y = 0;
path = [x+xp;y+yp];

% kinematic parameters
K =331/sqrt(3);
l1=166;
l2=110;
R=75.06;
param=[K,l1,l2,R];

% get joint angles initial value

[th10,th20,th30] = ikm(param,xini,yini,alphaini);
theta = [th10,th20,th30];
[ph10,ph20,ph30,psi10,psi20,psi30] = ikm_phi_psi(param,theta,xini,yini,alphaini);

% calculate joint angles for given path
% the joint angle for each kinematic chain will be stored in the array
% leg1,2,3
leg1 = zeros(3,length(path));
leg2 = zeros(3,length(path));
leg3 = zeros(3,length(path));

for i=1:length(path)
    % calculate joint angles using inverse kinematic
    [th1,th2,th3] = ikm(param,path(1,i),path(2,i),0);
    theta = [th1,th2,th3];
    [ph1,ph2,ph3,psi1,psi2,psi3] = ikm_phi_psi(param,theta,path(1,i),path(2,i),0);
    % here we assume that the robot always keeps the same working mode
    leg1(:,i) = [th1(wm1),ph1(wm1),psi1(wm1)];
    leg2(:,i) = [th2(wm2),ph2(wm2),psi2(wm2)];
    leg3(:,i) = [th3(wm3),ph3(wm3),psi3(wm3)];

end

% Apply the Virtual Work Principle

dU = zeros(1,36);
k = 3.16; % stiffness is 3.16 N.m/rad
% calculate total potential elastic energy
for i=1:length(path)
    % calculate 1/2*k*(th-th0)^2
    d1 = 1/2*k*(leg1(:,i)-[th10(wm1);ph10(wm1);psi10(wm1)]).^2;
    d2 = 1/2*k*(leg2(:,i)-[th20(wm2);ph20(wm2);psi20(wm2)]).^2;
    d3 = 1/2*k*(leg3(:,i)-[th30(wm3);ph30(wm3);psi30(wm3)]).^2;
    % sum into total energy
    dU(i) = sum(d1)+sum(d2)+sum(d3);
    
end

% apply discrete differentiation of dU and theta1,2,3
% theta1,2,3 is stored as the first angle 
T1 = diff(dU) / diff(leg1(1,:));
T2 = diff(dU) / diff(leg2(1,:));
T3 = diff(dU) / diff(leg3(1,:));

