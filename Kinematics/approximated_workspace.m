%% This program plots the approximated workspace

% kinematic parameters
K=331/sqrt(3);
l1=166;
l2=110;
R=75.06;
param=[K,l1,l2,R];

% actuator position
xA=K*cos(pi/6);
yA=K*sin(pi/6);

% draw the approximated workspaces from circles intersection
figure
hold on
% draw the circles centred around the actuator and of radius (l1+l2+R)
circle(-xA,-yA,l1+l2+R,'r')
circle(xA,-yA,l1+l2+R,'b')
circle(0,K,l1+l2+R,'g')
% draw the circles centred around the actuator and of radius (l1-(l2+R))
circle(-xA,-yA,abs(l1-l2-R),'r')
circle(xA,-yA,abs(l1-l2-R),'b')
circle(0,K,abs(l1-l2-R),'g')
% add the origin and center points
plot(0,0,'k+')
plot(-xA,-yA,'xr')
plot(xA,-yA,'xb')
plot(0,K,'xg')
% and the annotations
text(10,10,'(0,0)')
text(-xA-60,-yA-10,'A1')
text(xA+20,-yA-10,'A2')
text(0,K+40,'A3')
hold off
% set plot limits and titles
xlim([-420,420])
xlabel('x (mm)')
ylim([-420,420])
ylabel('y (mm)')
pbaspect([1 1 1])
legend('1st kinematic chain','2nd kinematic chain','3rd kinematic chain')
title("Workspace Approximation based on Kinematic Lengths")


% Obtain the approximated workspace as a polyshape
% once again, get the circles centred around each actuator
p1 = polycircle([-xA,-yA],l1+l2+R);
p2 = polycircle([-xA,-yA],l1-l2-R);
p3 = polycircle([xA,-yA],l1+l2+R);
p4 = polycircle([xA,-yA],l1-l2-R);
p5 = polycircle([0,K],l1+l2+R);
p6 = polycircle([0,K],l1-l2-R);
% and obtain their intersection
p = intersect(p1,p3);
p = intersect(p,p5);
p = subtract(p,p2);
p = subtract(p,p4);
p = subtract(p,p6);

% plot the polyshape figure
figure
hold on
plot(p)
% add annotations
plot(0,0,'k+')
text(10,10,'(0,0)')
hold off
% set plot limits and title
xlim([-220,220])
xlabel('x (mm)')
ylim([-220,220])
ylabel('y (mm)')
pbaspect([1 1 1])
title("Approximated Workspace")

