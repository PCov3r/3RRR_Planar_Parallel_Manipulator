function th = ikm(param,x,y,alpha)
% kinematic parameters
K = param(1);
l1 = param(2);
l2 = param(3);
R = param(4);

% actuators and platform vertices positions
i = 1:3;
xA = K*cos(2*i*pi/3+pi/2);
yA = K*sin(2*i*pi/3+pi/2);
xCP = R*cos(alpha+2*i*pi/3-pi/2);
yCP = R*sin(alpha+2*i*pi/3-pi/2);

% calculate ai, bi and ci as expressed in the report
a = K^2+R^2+l1^2-l2^2+x^2+y^2+2*xA.*xCP-2*xA.*x-2*xCP.*x+2*yA.*yCP-2*yA.*y-2*yCP.*y;
b = 2*l1*(xA+xCP-x);
c = 2*l1*(yA+yCP-y);

% obtain the elbow-up solutions
thp = 2*atan((-c+sqrt(c.^2+b.^2-a.^2)) ./ (a-b));
% obtain the elbow-down solutions
thm = 2*atan((-c-sqrt(c.^2+b.^2-a.^2)) ./ (a-b));

% arrange answer in the form [th1_up,th1_down,th2up,th2_down...]
th = reshape([thp;thm], size(thp,1), []);

end

% cph = xA-l1*cos([thp;thm])-R*cos(alpha+(4*i-3)*pi/6)+x;
% sph = yA-l1*sin([thp;thm])-R*sin(alpha+(4*i-3)*pi/6)+y;
% ph = atan(sph(1,:),cph(1,:));

% cph1 = xA-l1*cos([thp(1);thm(1)])-R*cos(alpha+pi/6)+x;
% sph1 = yA-l1*sin([thp(1);thm(1)])-R*sin(alpha+pi/6)+y;
% cph2 = -xA-l1*cos([thp(2);thm(2)])-R*cos(alpha+5*pi/6)+x;
% sph2 = yA-l1.*sin([thp(2);thm(2)])-R*sin(alpha+5*pi/6)+y;
% cph3 = -(l1.*cos([thp(3);thm(3)])+R*cos(alpha-pi/2)-x);
% sph3 = -(K+l1.*sin([thp(3);thm(3)])+R*sin(alpha-pi/2)-y);
% 
% ph1 = atan2(sph1,cph1);
% ph2 = atan2(sph2,cph2);
% ph3 = atan2(sph3,cph3);

% ph = [ph1,ph2,ph3];

%%% Solve for psi
% psi1 = pi-ph1+pi/6+alpha;
% psi2 = pi-ph2+5*pi/6+alpha;
% psi3 = pi-ph3+pi/2+alpha;

% psi = [psi1,psi2,psi3];
