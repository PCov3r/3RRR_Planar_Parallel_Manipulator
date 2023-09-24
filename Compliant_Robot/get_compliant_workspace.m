function [comp_workspace] = get_compliant_workspace(param, limit, home_pos, mode, orientation)

% home position
xini = home_pos(1);
yini = home_pos(2);
alphaini = home_pos(3);

% working mode
mode = regexprep(mode,{'+','-'},{'1','2'});
working_mode = str2double(split(mode));
% get working mode for each kinematic chain
wm1 = working_mode(1);
wm2 = working_mode(2);
wm3 = working_mode(3);


% initial angles
[th10,th20,th30] = ikm(param,xini,yini,alphaini);

theta = [th10,th20,th30];
[ph10,ph20,ph30,psi10,psi20,psi30] = ikm_phi_psi(param,theta,xini,yini,alphaini);

% obtain 1st kinematic chain reachable area %

% sweep across all possible values of theta and phi (initial angle +- max)
theta = linspace(th10(wm1)-limit,th10(wm1)+limit);
phi = linspace(ph10(wm1)-limit,ph10(wm1)+limit);
% calculate psi from phi
psi = pi-phi+pi/6+alphaini+orientation;
% find value of psi greater than limit
idx = find(angdiff(psi,psi10(wm1))>limit);
% delete values of phi that lead to angle limit violation
phi(idx) = [];

% obtain the reachable points
coord1 = [];
for t=theta
    for p=phi
        coord1 = [coord1,get_coord(param,t,p,alphaini,2*pi/3)];
    end
end

% obtain 2nd kinematic chain reachable area %

% sweep across all possible values of theta and phi (initial angle +- max)
theta = linspace(th20(wm2)-limit,th20(wm2)+limit);
phi = linspace(ph20(wm2)-limit,ph20(wm2)+limit);
% calculate psi from phi
psi = pi-phi+5*pi/6+alphaini+orientation;
% find value of psi greater than limit
idx = find(angdiff(psi,psi20(wm2))>limit);
% delete values of phi that lead to angle limit violation
phi(idx) = [];

% obtain the reachable points
coord2 = [];
for t=theta
    for p=phi
        coord2 = [coord2,get_coord(param,t,p,alphaini,-2*pi/3)];
    end
end

% obtain 3rd kinematic chain reachable area %

% sweep across all possible values of theta and phi (initial angle +- max)
theta = linspace(th30(wm3)-limit,th30(wm3)+limit);
phi = linspace(ph30(wm3)-limit,ph30(wm3)+limit);
% calculate psi from phi
psi = pi-phi+3*pi/2+alphaini+orientation; 
% find value of psi greater than limit
idx = find(angdiff(psi,psi30(wm3))>limit);
% delete values of phi that lead to angle limit violation
phi(idx) = [];

% obtain the reachable points
coord3 = [];
for t=theta
    for p=phi
        coord3 = [coord3,get_coord(param,t,p,alphaini,0)];
    end
end


%%%%%  PLOT %%%%%
% obtain the boundary of the reachable area
k = boundary(coord1(1,:)',coord1(2,:)');
% and transform the boundary in a polyshape figure
wrkspace1 = polyshape(coord1(1,k),coord1(2,k));
% repeat for the other kinematic chains
k = boundary(coord2(1,:)',coord2(2,:)');
wrkspace2 = polyshape(coord2(1,k),coord2(2,k));
k = boundary(coord3(1,:)',coord3(2,:)');
wrkspace3 = polyshape(coord3(1,k),coord3(2,k));

% obtain workspace from intersection
figure
hold on
plot(wrkspace1)
plot(wrkspace2)
plot(wrkspace3)
hold off
legend('1st','2nd','3rd')
axis equal
comp_workspace = intersect([wrkspace1,wrkspace2,wrkspace3]);

end
