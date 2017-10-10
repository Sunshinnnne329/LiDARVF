close all;
clear variables;

load funcList

r=1;
G=1;
H=1;
L=1;

for i = 1:length(funcList)
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
    opt.DecayFunc = funcList(i).func;
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
    set(gcf, 'Color', 'w')    
    xlabel('X-Position [-]');
    ylabel('Y-Position [-]');
    title(funcList(i).name);
    %export_fig(gcf, ['ObsticleField_' funcList(i).name], '-m3');

    figure;
    r = 0:0.01:1;
    plot(r,funcList(i).func(r),'LineWidth',2);
    grid on;
    set(gcf, 'Color', 'w')
    xlabel('Radius Ratio [-]');
    ylabel('Activation Level [-]');
    title(funcList(i).name);
    %export_fig(gcf, ['RadialProfile_' funcList(i).name], '-m3');
end