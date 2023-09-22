%% This function returns a polyshape circle
function poly=polycircle(center,radius)

n=1000;
theta = (0:n-1)*(2*pi/n);
x = center(1) + radius*cos(theta);
y = center(2) + radius*sin(theta);
poly = polyshape(x,y);

end
