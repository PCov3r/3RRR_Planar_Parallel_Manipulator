function [dJth,dJx] = det_jacobian(param,theta,phi,alpha)
% kinematic parameters
K = param(1);
L1 = param(2);
L2 = param(3);
R = param(4);

% active joint angles
th1 = theta(1);
th2 = theta(2);
th3 = theta(3);
    
% passive joint angles
ph1 = phi(1);
ph2 = phi(2);
ph3 = phi(3);

% calculate Jacobians
dJth = L1^3*sin(ph1-th1)*sin(ph2-th2)*sin(ph3-th3);

Jx = [-cos(ph1), -sin(ph1), R*sin(ph1-alpha-pi/6);
      -cos(ph2), -sin(ph2), R*sin(ph2-alpha-5*pi/6);
      -cos(ph3), -sin(ph3), R*sin(ph3-alpha-3*pi/2)];
dJx = det(Jx);

end
