%% Test of the inverse kinematics 
clear all

K =331/sqrt(3);
l1=166;
l2=110;
R=75.06;

param=[K,l1,l2,R];

% Robot position
x = 0;
y = 0;
alpha = 0;

% Inverse kinematics
[th1,th2,th3] = ikm(param,x,y,alpha);
theta = [th1,th2,th3];
[ph1,ph2,ph3,psi1,psi2,psi3] = ikm_phi_psi(param,theta,x,y,alpha);
phi = [ph1,ph2,ph3];
psi = [psi1,psi2,psi3];

% show angle in degrees wrapped around [0,360°]
rad2deg(mod(theta,2*pi))
rad2deg(mod(phi,2*pi))
