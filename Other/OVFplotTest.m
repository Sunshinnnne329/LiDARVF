clc;
clear all;
close all;

avoidVF = {};
[avoidVF, ~] = makeOVF(0, 0, 0.01, avoidVF);
[avoidVF, opt] = makeOVF(0, 1, 0.05, avoidVF);

%% Plot VFs
figure;
hold on;

for ii=1:length(avoidVF)
    opt.bCustomRange = avoidVF{ii}.plotrange;
    opt.bCustomCenter = avoidVF{ii}.plotcenter;
    opt.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
    opt.Color = [1 0 0];
    RET_temp = avoidVF{ii}.VF.PlotFieldAroundRadius(gca,opt);
    plot_h_avoid = RET_temp.H;
end

axis equal;
grid on;
xlabel('X-Position [-]');
ylabel('Y-Position [-]');
