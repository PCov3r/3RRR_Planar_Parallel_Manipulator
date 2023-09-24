%% Get position of end effector from theta, phi and alpha
%% The actuator is assumed to be at the top in A3
%% rot is the angle of the other actuators: 2pi/3 for A1 and -2pi/3 for A2

function end_effector = get_coord(param,theta,phi,alpha,rot)
% kinematic parameters
K =param(1);
l1=param(2);
l2=param(3);
R=param(4);

% rotation matrix
rot_mat = [ cos(rot)   -sin(rot);
            sin(rot)    cos(rot) ];

% vectors coordinates
OA = rot_mat*[0;K];
AB = [l1*cos(theta);l1*sin(theta)];
BC = [l2*cos(phi);l2*sin(phi)];
CP = rot_mat*[R*cos(alpha-pi/2);R*sin(alpha-pi/2)];

% end-effector's final position
end_effector = OA+AB+BC+CP;

end
