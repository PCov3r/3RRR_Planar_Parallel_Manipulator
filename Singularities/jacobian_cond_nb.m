function [dJth,dJx] = jacobian_cond_nb(theta,phi,alpha,param)
% kinematic parameters
K = param(1);
l1 = param(2);
l2 = param(3);
R = param(4);

% theta1,2,3
th1 = theta(:,1);
th2 = theta(:,2);
th3 = theta(:,3);
% phi1,2,3
ph1 = phi(:,1);
ph2 = phi(:,2);
ph3 = phi(:,3);

% calculate Jtheta condition number
Jth = [l1*sin(ph1-th1)  0               0               ;
        0               l1*sin(ph2-th2) 0               ;
        0               0               l1*sin(ph3-th3) ];

condJth = cond(Jth,1);
% inverse condition number
dJth = 1/condJth;

% calculate Jx condition number
Jx = [-cos(ph1), -sin(ph1), R*sin(ph1-alpha-pi/6);
      -cos(ph2), -sin(ph2), R*sin(ph2-alpha-5*pi/6);
      -cos(ph3), -sin(ph3), R*sin(ph3-alpha-3*pi/2)];

condJx = cond(Jx,1);
% inverse condition number
dJx = 1/condJx;

end
