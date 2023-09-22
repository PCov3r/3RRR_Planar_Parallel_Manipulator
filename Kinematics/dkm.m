function [x,y,alpha] = dkm2(param, th1,th2,th3)
% kinematic parameters
K = param(1);
l1 = param(2);
l2 = param(3);
R = param(4);

% symbolic expressions of alpha, phi2 and ta=alpha/2
syms a ph2 ta 

% kinematic loop 1: dk1 is equal to l2*cos(phi1) and dk2 to l2*sin(phi1)
dk1 = l1*cos(th1)+sqrt(3)*R*cos(a)-l2*cos(ph2)-l1*cos(th2)-sqrt(3)*K;
dk2 = l1*sin(th1)+sqrt(3)*R*sin(a)-l2*sin(ph2)-l1*sin(th2);

% get rid of dependency in phi1
% expand the squared dk1 and dk2, simplify expressions in cos^2+sin^2 and
% expand again to obtain an expression in cos(ph2) and sin(ph2)
loop1 = expand(simplify(expand((dk1)^2+(dk2)^2-l2^2)));

% obtain f1, f2 and f3 from f1*cos(ph2)+f2*sin(ph2)+f3=0 using subtitutions
f3 = subs(loop1,{cos(ph2),sin(ph2)},{0,0});
f1 = subs(loop1,{cos(ph2),sin(ph2)},{1,0})-f3;
f2 = subs(loop1,{cos(ph2),sin(ph2)},{0,1})-f3;

% kinematic loop 2: dk3 is equal to l2*cos(ph3) and dk4 to l2*sin(ph3)
dk3 = l1*cos(th3)-sqrt(3)*R*cos(a+2*pi/3)-l2*cos(ph2)-l1*cos(th2)+sqrt(3)*K*cos(2*pi/3);
dk4 = l1*sin(th3)-sqrt(3)*R*sin(a+2*pi/3)-l2*sin(ph2)-l1*sin(th2)+sqrt(3)*K*sin(2*pi/3);

% get rid of dependency in phi3
% expand the squared dk3 and dk4, simplify expressions in cos^2+sin^2 and 
% expand again to obtain an expression in cos(ph2) and sin(ph2)
loop2 = expand(simplify(expand((dk3)^2+(dk4)^2-l2^2)));
g3 = subs(loop2,{cos(ph2),sin(ph2)},{0,0});
g1 = subs(loop2,{cos(ph2),sin(ph2)},{1,0})-g3;
g2 = subs(loop2,{cos(ph2),sin(ph2)},{0,1})-g3;

% eliminate phi2 using:
% (f2g1-f1g2)^2*(cos(phi2)^2+sin(phi2)^2)=(f1*g3-f3*g1)^2+(g2*f3-f2*g3)^2
u = (expand((f2*g1-f1*g2)^2));
v = (expand((g2*f3-f2*g3)^2));
w = (expand((f1*g3-f3*g1)^2));

% express u-v-w=0 using half-angle subs
eq_alpha = subs(u-v-w,{cos(a),sin(a)},{(1-ta^2)/(1+ta^2),2*ta/(1+ta^2)});

% assume ta to be real and solve for ta
assume(ta,'real')
sol_ta = vpasolve(eq_alpha,ta,[-Inf Inf]); 
% obtain alpha from ta
alpha = double(2*atan(sol_ta));

% express cos(phi2) and sin(phi2) from alpha and solve for phi2
sphi2 = double(subs((f1*g3-f3*g1)/(f2*g1-f1*g2),a,alpha));
cphi2 = double(subs((g2*f3-f2*g3)/(f2*g1-f1*g2),a,alpha));
phi2 = atan2(sphi2,cphi2);

% obtain phi1 and phi3 using the same method
sphi1 = double(subs(-dk2,{a,ph2},{alpha,phi2}));
cphi1 = double(subs(-dk1,{a,ph2},{alpha,phi2}));
phi1 = atan2(sphi1,cphi1);

sphi3 = double(subs(-dk4,{a,ph2},{alpha,phi2}));
cphi3 = double(subs(-dk3,{a,ph2},{alpha,phi2}));
phi3 = atan2(sphi3,cphi3);

% obtain x and y using third kinematic chain equations
x = l1*cos(th3)+l2*cos(phi3)-R*cos(alpha+pi/2);
y = K+l1*sin(th3)+l2*sin(phi3)-R*sin(alpha+pi/2);


end
