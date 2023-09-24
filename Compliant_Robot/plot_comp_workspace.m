%% This program is used for the plotting of the compliant workspace
clear all

% kinematic parameters
K=331/sqrt(3);
l1=166;
l2=110;
R=75.06;
param=[K,l1,l2,R];

% compliant joint limitation
limit=13.4*pi/180;
% working mode
mode = '+ + +';
% home position
home_pos=[0,0,0];
% orientation of the end-effector
orientation = 0*pi/180;

% obtain the compliant workspace
comp_workspace = get_compliant_workspace(param, limit, home_pos, mode, orientation);

% plot the workspace
figure
plot(comp_workspace)
