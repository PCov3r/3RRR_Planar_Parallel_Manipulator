%% This program is used to plot the singularity loci of the 3-RRR robot

% nb_points: number of points used for discretization
% alpha: platform's orientation
nb_points = 400;
alpha  = 0;

% kinematic parameters
K =331/sqrt(3);
l1=166;
l2=110;
R=75.06;

param=[K,l1,l2,R];

% set of discrete coordinates (here x=[-210;210], y=[-210;210])
coord = linspace(-210,210,nb_points);

% initialize loci matrices (8 in total, for each working mode)
detJth = zeros(8,width(coord),width(coord));
detJx = zeros(8,width(coord),width(coord));

% iterate through coordinates and obtain singularity loci
for j=1:length(coord)
    for k=1:length(coord)
        j,k
        % calculate theta1,2,3 for given coordinate
        [th1,th2,th3] = ikm(param,coord(j),coord(k),alpha);
        % if solution exists (i.e., if the point is part of the workspace)
        if(isreal(th1) && isreal(th2) && isreal(th3))
            % calculate determinants for Jth and Jx for each possible
            % working mode
            [dJth, dJx] = combine_det(param,th1,th2,th3,coord(j),coord(k),alpha);
            % save determinant values
            detJth(:,k,j) = dJth;
            detJx(:,k,j) = dJx;
        end 
    end
end

% normalize the determinants
Mx = max(max(max(abs(detJx))));
detJx = detJx/Mx;

Mth = max(max(max(abs(detJth))));
detJth = detJth/Mth;

% Plot the final singularity loci
figure
test = imagesc(abs(squeeze(detJx(1,:,:))));
axis image
c = colorbar();
c.Label.String = 'Absolute value of det(Jx) (normalized)';
set(gca,'YDir','normal') 
title('Direct (type II) singularity contours')
set(gca, 'XTick', linspace(10*nb_points/840,nb_points-10*nb_points/840,9), 'XTickLabel', linspace(-200,200,9)); % 10 ticks 
set(gca, 'YTick', linspace(10*nb_points/840,nb_points-10*nb_points/840,9), 'YTickLabel', linspace(-200,200,9)); % 20 ticks
xlabel('x (mm)')
ylabel('y (mm)')


%% Helper functions

% the function 'combine_det' is used to obtain the jacobian determinants
% for each working mode
function [detJth, detJx] = combine_det(param,th1,th2,th3,x,y,a)
    % start by combining the input angles theta1,2,3 into the 8 possible
    % working modes using the ndgrid function. 
    % The function ndgrid outputs every possible combinations of vectors,
    % arranging them in a list. 
    % e.g., for A=[1 2] and B=[3 4], [a,b]=ndgrid(A,B)
    % will output a=[1 1; 2 2] and b=[3 4; 3 4] so that when we combine a
    % and b, we have [1 3; 1 4; 2 3; 2 4], which are all the possible
    % combinations of A and B.
    [TH1,TH2,TH3] = ndgrid(th1,th2,th3);
    comb = [TH1(:) TH2(:) TH3(:)];

    % initiate the value of each working mode determinant to 0.
    detJx = zeros(8,1);
    detJth = zeros(8,1);

    % go through all the combinations (=working modes)
    for k=1:length(comb)
        th1 = comb(k,1);
        th2 = comb(k,2);
        th3 = comb(k,3);
        % calculate the associated phi1,2,3
        [ph1,ph2,ph3] = ikm_phi_psi(param,[th1,th2,th3],x,y,a);
        th = [th1,th2,th3];
        ph = [ph1,ph2,ph3];
        % calculate determinants
        [Jth,Jx] = det_jacobian(param,th,ph,a);
        % save determinant for particular working mode
        detJth(k) = Jth;
        detJx(k) = Jx;
    end
end
