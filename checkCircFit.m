function [bool,x_circ,y_circ,R] = checkCircFit(indexed_x,indexed_y)

[xc,yc,R] = circfit(indexed_x,indexed_y);
theta = linspace(0,atan2(indexed_y(end)-indexed_y(1),indexed_x(end)-indexed_x(1)),length(indexed_x));
x_circ = xc+R*cosd(theta);
y_circ = yc+R*sind(theta);

x_check = find(indexed_x == x_circ);
y_check = find(indexed_y == y_circ);

a = sqrt((indexed_x(1)-indexed_x(2))^2+(indexed_y(1)-indexed_y(2))^2);

r= a*(1/sin(pi/length(indexed_x)))

if abs(r - R) < 1
    bool = 1;
else
    bool = 0;
end
end


