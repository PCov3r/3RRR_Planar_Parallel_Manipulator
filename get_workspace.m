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
% get circles associated with 2nd kinematic chain (rotation of -120°°
circle21 = polycircle(R2*(OA3+C3P),L1+L2);
circle22 = polycircle(R2*(OA3+C3P),abs(L1-L2));
% (polycircle is a custom function, please refer to GitHub)

% get intersection of exterior circles (external boundaries)
polyout = intersect([circle11,circle21,circle31]);
% subtract interior circles (internal boundaries)
polyout = subtract(polyout,circle12);
polyout = subtract(polyout,circle22);
total_wrk = subtract(polyout,circle32);
end


% xA=K*cos(pi/6);
% yA=K*sin(pi/6);
% 
% b=R*cos(pi/6);
% c=R*sin(pi/6);
% 
% figure
% hold on
% circle1 = subtract(circle1,circle2);
% circle1 = subtract(circle1,circle4);
% circle1 = subtract(circle1,circle6);
% plot(circle1)
% circle3 = subtract(circle3,circle2);
% circle3 = subtract(circle3,circle4);
% circle3 = subtract(circle3,circle6);
% plot(circle3)
% circle5 = subtract(circle5,circle2);
% circle5 = subtract(circle5,circle4);
% circle5 = subtract(circle5,circle6);
% plot(circle5)
% plot(-xA,-yA,'xk')
% plot(xA,-yA,'xk')
% plot(0,K,'xk')
% text(-xA-40,-yA-10,'A1')
% text(xA+20,-yA-10,'A2')
% text(0,K+20,'A3')
% 
% plot(-xA+R*cos(pi/6),-yA+R*sin(pi/6),'xk')
% plot(xA-R*cos(pi/6),-yA+R*sin(pi/6),'xk')
% plot(0,K-R,'xk')
% text(-xA+30,-yA+70,'A1+C1P')
% text(xA-100,-yA+70,'A2+C2P')
% text(-30,K-100,'A3+C3P')
% hold off
% xlabel("x (mm)")
% ylabel("y (mm)")
% axis equal
% title('Real Workspace')