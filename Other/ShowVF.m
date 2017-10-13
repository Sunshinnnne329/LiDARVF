close all;
clear variables;
r=1;
G=1;
H=1;
L=1;
cVF = CircleVectorField('Gradient',r);
cVF.G=G;
cVF.H=H;
cVF.L=L;
cVF.xc=0;
cVF.yc=0;
cVF.vel_x=1;
cVF.vel_y=0;
%cVF.velPathFunc = @vPath;
cVF.bUsePathFunc = ~true;
cVF.bUseVRel = ~true;
cVF.mLegendName = '';

clear opt;
opt.bShowCircle=true;
opt.bPlotQuiverNorm=true;
opt.bNormVFVectors=true;
%opt.DecayFunc = @NoVDecay;
opt.Color = [0 0 1];
opt.sOnlyShow = 'Circ';
opt.sOnlyShow = 'Conv';
opt.sOnlyShow = 'TV';
types = {'Circ','Conv','TV','All'};
opt.CustomNumberOfPoints=25;
for ii=1:length(types)
    opt.sOnlyShow = types{ii};
    figure;hold all;
    RET_VF = cVF.PlotFieldAroundRadius(gca,opt);
    axis equal;
    grid on;
    xlabel('X-Position [-]');
    ylabel('Y-Position [-]');
    saveas(gcf,types{ii},'epsc')
end

clear opt;
oVFa = CircleVectorField('Gradient',0.01);
oVFa.G=-G;
oVFa.H=0;
oVFa.L=0;
oVFa.xc=0;
oVFa.yc=1.5;
% oVFa.vel_x=velx_t;
% oVFa.vel_y=vely_t;
oVFa.velPathFunc = @vPath;
oVFa.bUsePathFunc = true;
oVFa.bUseVRel = ~true;
oVFa.mLegendName = 'Obstacle 1';

oVF1.VF = oVFa;
oVF1.plotcenter = [0,1.5];
oVF1.plotrange = 0.75;
oVF1.plotradius = 0.5;
opt.bNormVFVectors=false;
opt.DecayFunc = @ VDecayTanh;%VDecay;
opt.bPlotQuiverNorm = true;
opt.bCustomRange = oVF1.plotrange;
opt.bCustomCenter = oVF1.plotcenter;
opt.bCustomCircleRadiusPlot = oVF1.plotradius;
opt.CustomNumberOfPoints=25;
opt.bShowCircle=true;
opt.Color = [1 0 0];
% opt.UAV = UAV_list{1};
figure;hold on;
RET_temp = oVF1.VF.PlotFieldAroundRadius(gca,opt);
plot_h_avoid = RET_temp.H;
axis equal;
grid on;
xlabel('X-Position [-]');
ylabel('Y-Position [-]');
% 
figure;
r = 0:0.01:1;
plot(r,VDecayTanh(r),'LineWidth',2);
grid on;
xlabel('Radius Ratio [-]');
ylabel('Activation Level [-]');

function G = VDecayTanh(rrin)
    rrt = -((rrin)*2*pi-pi);
%     G = tanh(rrt)/2+0.5;
    G = (tanh(rrt)+1)/2;

end