clc;
close all;
clear variables;


cVF = CircleVectorField('Gradient');
cVF.G=1;
cVF.H=1;
cVF.L=1;
cVF.xc=0;
cVF.yc=0;
cVF.vel_x=0;
cVF.vel_y=0;

lVF = CircleVectorField('Lyapunov');
lVF.xc=0;
lVF.yc=0;
lVF.vel_x=0;
lVF.vel_y=0;

rd = 1; %loiter radius


limit = rd*1.5;
step = 0.5;
x_list = linspace(-limit,limit,50);%-limit:step:limit;
y_list = linspace(-limit,limit,50);
for i=1:length(x_list)
    x = x_list(i);
    for ii=1:length(y_list)
        y = y_list(ii);
        r = sqrt(x^2+y^2);
        u = -x*(r^2-rd^2)/(r^2+rd^2) - y*(2*r*rd)/(r^2+rd^2);
        v = -y*(r^2-rd^2)/(r^2+rd^2) + x*(2*r*rd)/(r^2+rd^2);
        
        u_list(i,ii)=u;
        v_list(i,ii)=v;
        xq(i,ii)=x;
        yq(i,ii)=y;
    end
end
figure;hold all;

opt.bShowCircle=~true;
opt.bNormVFVectors=true;
opt.bPlotQuiverNorm=true;
cVF.PlotFieldAroundRadius(gca,opt,[]);
lVF.PlotFieldAroundRadius(gca,opt,[]);
% 
% u_list=u_list./sqrt(u_list.^2+v_list.^2);
% v_list=v_list./sqrt(u_list.^2+v_list.^2);
% quiver(gca,xq,yq,u_list,v_list);

th = 0:pi/50:2*pi;
x_c=0;
y_c=0;
xunit = rd * cos(th) + x_c;
yunit = rd * sin(th) + y_c;
h = plot(xunit, yunit);
axis equal;