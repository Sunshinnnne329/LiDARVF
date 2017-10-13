function [list, opt] = makeOVF(xc,yc,r_in,list)

%% Create vector field object using Goncalves method
oVFa = CircleVectorField('Gradient',r_in);
oVFa.G=1;  % Convergence weight
oVFa.H=0;   % Circulation weight
oVFa.L=0;   % Time varying weight
oVFa.xc=xc;  % VF center x-component
oVFa.yc=yc;  % VF center y-component
oVFa.mLegendName = 'Obstacle';

%% VF plot options
oVF1.VF = oVFa; 
oVF1.plotcenter = [oVFa.xc,oVFa.yc];
oVF1.plotrange = 1.5;
oVF1.plotradius = r_in;
bShowVectorField= true;
opt.bNormVFVectors = true;
opt.bPlotQuiverNorm = true;
opt.bShowCircle = true;
opt.CustomNumberOfPoints=25;
opt.DecayFunc = @decayFunc;%VDecay;

%% Add VF to list of VFs to be plotted (how Jay creates multiple fields)
list = {list{:},oVF1};

%% Decay functions go here
function G = decayFunc(r)
R = 2;
G = -(1/R).*r + 1;
nullme = find(G < 0);
G(nullme) = zeros(1,length(nullme));
end

end
