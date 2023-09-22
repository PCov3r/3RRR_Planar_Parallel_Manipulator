%% This function is an implementation of the inverse kinematics solution
%% It returns theta1 theta2 and theta3 using the robot position x y and orientation alpha

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
