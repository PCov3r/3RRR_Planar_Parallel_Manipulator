function total_wrk = get_workspace(alpha,param)
% kinematic parameters
K =param(1);
L1=param(2);
L2=param(3);
R=param(4);

% rotation matrix of +120°
R1 = [  -1/2      -sqrt(3)/2;
      sqrt(3)/2      -1/2    ];
% rotation matrix of -120°
R2 = [  -1/2       sqrt(3)/2;
      -sqrt(3)/2     -1/2    ];
% vectors OA3 and C3P
OA3 = [0; K];
C3P = [R*cos(alpha-pi/2); R*sin(alpha-pi/2)];

% get circles associated with 3rd kinematic chain
circle31 = polycircle(OA3+C3P,L1+L2);
circle32 = polycircle(OA3+C3P,abs(L1-L2));
% get circles associated with 1st kinematic chain (rotation of +120°)
circle11 = polycircle(R1*(OA3+C3P),L1+L2);
circle12 = polycircle(R1*(OA3+C3P),abs(L1-L2));
% get circles associated with 2nd kinematic chain (rotation of -120°)
circle21 = polycircle(R2*(OA3+C3P),L1+L2);
circle22 = polycircle(R2*(OA3+C3P),abs(L1-L2));
% (polycircle is a custom function to obtain a polyshape circle: https://www.mathworks.com/help/matlab/ref/polyshape.html)

% get intersection of exterior circles (external boundaries)
polyout = intersect([circle11,circle21,circle31]);
% subtract interior circles (internal boundaries)
polyout = subtract(polyout,circle12);
polyout = subtract(polyout,circle22);
total_wrk = subtract(polyout,circle32);
end
