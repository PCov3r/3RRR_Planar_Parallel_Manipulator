% This function returns the maximum allowed joint rotation to accomodate
% for a singularity-free workspace.

% param: the kinematic parameters
% loci: the singularity loci on which the optimization is based (see example)
% threshold: the singularity threshold, a point with a determinant below 
% this threshold is considered to be a singularity.

function max_limit = optimize_workspace(param,loci,threshold)
% kinematic parameters
K = param(1);
l1 = param(2);
l2 = param(3);
R= param(4);

% find the coordinates associated with a singularity
[idx,idy]=find(loci< threshold);
% our loci is an image and therefore, the pixel position needs to be 
% converted to coordinates. Our loci are calculated for [-210,210]*[-210,210]

% Get the image size
[rows, columns] = size(loci);
% Create the spatial calibration factors
pixel2xcoord = (210 - -210) / columns;
pixel2ycoord = (210 - -210) / rows;
% convert the pixel indexes to real coordinates
idx = -210 + pixel2xcoord * idx;
idy = -210 + pixel2ycoord * idy;

% Optimize the workspace %

% create an array containing the possible joint limits
joint_limit = linspace(1,180,180);
% create a variable to store the max allowed joint limit
max_limit = 0;

% we will iterate through each possible joint limit.
% We will calculate the compliant workspace and test if it contains a singularity

i = 1;
% calculate compliant workspace for first joint limit (1°)
comp_workspace = get_compliant_workspace(param,joint_limit(i)*pi/180,[0,0,0],'+ + +',0);
% test if the compliant workspace contains a singularity, using their coordinates
test = isinterior(comp_workspace,idx,idy);

% sweep through the joint limits until the workspace contains a singularity
while(~any(test) && i<length(joint_limit))
    % update maximum allowed joint limit
    max_limit = joint_limit(i);
    % increment current joint limit and get new workspace
    i = i+1;
    comp_workspace = get_compliant_workspace(param,joint_limit(i)*pi/180,[0,0,0],'+ + +',0);
    % test if new workspace contains a singularity
    test = isinterior(comp_workspace,idx,idy);
end

end
