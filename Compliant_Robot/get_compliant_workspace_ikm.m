%% This function outputs the compliant workspace based on the inverse kinematics method.
% param: kinematic parameters
% limit: joint limitation (radians)
% home_pos: compliant robot home position
% mode: compliant robot initial working mode
% orientation: platform's orientation

function [comp_workspace] = get_compliant_workspace_ikm(param, limit, home_pos, mode, orientation)

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


% Workspace analysis %

% set joint limitation
max_angle = limit;

% set coordinates to sweep through
coord = linspace(-50,50,400);
% set platform's orientation
alpha = alphaini+orientation; 
wrkspace = [];
% sweep through all coordinates
for i=1:length(coord)
    for j=1:length(coord)
        % inverse kinematics, theta1,2,3
        [th1,th2,th3] = ikm(param,coord(i),coord(j),alpha);
        % if point is part of the rigid-body workspace
        if(isreal(th1) && isreal(th2) && isreal(th3))
            % if all angles are less than the limitation compared to the initial angles
            if testAngle(angdiff(th1,th10(wm1)),angdiff(th2,th20(wm2)),angdiff(th3,th30(wm3)),max_angle)
                % inverse kinematics, phi1,2,3 and psi1,2,3
                [ph1,ph2,ph3,psi1,psi2,psi3] = ikm_phi_psi(param,[th1,th2,th3],coord(i),coord(j),alpha);
                % test joint limitation
                if testAngle(angdiff(ph1,ph10(wm1)),angdiff(ph2,ph20(wm2)),angdiff(ph3,ph30(wm3)),max_angle)
                    if testAngle(angdiff(psi1,psi10(wm1)),angdiff(psi2,psi20(wm2)),angdiff(psi3,psi30(wm3)),max_angle)
                        % all joints passed the test, point is part of the
                        % compliant workspace
                        wrkspace = cat(1,wrkspace, [coord(i),coord(j)]);
                    end
                end
            end
        end
    end
end


%% plot workspace
% uncomment the following lines if you want to plot the raw workspace

% hold on
%     scatter(wrkspace(:,1),wrkspace(:,2),'.')
% hold off
% title("Compliant Workspace - Kinematics-based method")
% axis equal
% xlabel("x (mm)")
% ylabel("y (mm)")
% xlim([-50,50])
% ylim([-50,50])

% obtain workspace boundaries from all the points and transform into polyshape.
% this might fail if workspace is empty.
try
    k = boundary(wrkspace(:,1),wrkspace(:,2));
    comp_workspace = polyshape(wrkspace(k,1),wrkspace(k,2));
catch
    warning('Could not find workspace boundary: too large orientation or coarse discretization')
    comp_workspace = polyshape();
end

%% User defined function

% This function test if all the angle differences are withing the limit
function bool = testAngle(diffang1,diffang2,diffang3,max)
    if(any(abs(diffang1)<=max) && any(abs(diffang2)<=max) && any(abs(diffang3)<=max))
        bool = true;
    else
        bool = false;
    end 
end

end
