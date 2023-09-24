%% This calculates the difference between the angles alpha and beta. 
%% This function subtracts alpha from beta with the result wrapped on theinterval [-pi,pi]. 
%% This allows to efficiently compare 2 angles.
%% For instance, 1° and 359° are supposed to be close, but a subtraction between the two would yield a wrong result

function angle_difference = angdiff(alpha,beta)
    angle_difference = wrapToPi(alpha-beta);
end
