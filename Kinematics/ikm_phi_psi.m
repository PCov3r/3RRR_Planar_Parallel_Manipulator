%% This function returns the expression of phi and psi with x, y, alpha and thetas as parameters.

function [ph1,ph2,ph3,psi1,psi2,psi3] = ikm_phi_psi(param,theta,x,y,alpha)
% kinematic parameters
K =param(1);
l1=param(2);
l2=param(3);
R=param(4);

% theta123
th1 = theta(:,1);
th2 = theta(:,2);
th3 = theta(:,3);
    
% actuator position
xA=K*cos(pi/6);
yA=K*sin(pi/6);

% define cos(phi) and sin(phi)
cph1 = (xA-l1*cos(th1)-R*cos(alpha+pi/6)+x)/l2;
sph1 = (yA-l1*sin(th1)-R*sin(alpha+pi/6)+y)/l2;
cph2 = (-xA-l1*cos(th2)-R*cos(alpha+5*pi/6)+x)/l2;
sph2 = (yA-l1*sin(th2)-R*sin(alpha+5*pi/6)+y)/l2;
cph3 = (-(l1*cos(th3)+R*cos(alpha-pi/2)-x))/l2;
sph3 = (-(K+l1*sin(th3)+R*sin(alpha-pi/2)-y))/l2;

% solve for phi using arctan2
ph1 = atan2(sph1,cph1);
ph2 = atan2(sph2,cph2);
ph3 = atan2(sph3,cph3);

% solve for psi
psi1 = pi-ph1+pi/6+alpha;
psi2 = pi-ph2+5*pi/6+alpha;
psi3 = pi-ph3+3*pi/2+alpha;

end
