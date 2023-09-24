function circle(x,y,r,color)
% x and y are the coordinates of the center of the circle
% r is the radius of the circle
% 0.01 is the angle step
ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp,color);
end
