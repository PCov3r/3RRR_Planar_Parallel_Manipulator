% This function is used to test the workspace optimization program.
% It supposes that the singularity loci have already been obtained.
% If that is not the case, please check singularity_loci.m

% Set the loci as type II singularity
% Notice that we first need to take the absolute value of the determinant,
% since it is ranging from -1 to 1.
% The squeeze function is used to resize the matrix from size 1*800*800 to 800*800
loci = squeeze(abs(detJx(1,:,:)));
% obtained the maximum allowed joint rotation by considering points with 
% det(Jx)<0.5 as singularities.
max_limit = optimize_workspace(param,loci,0.5);
