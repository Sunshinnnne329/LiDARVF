clc;
close all;
clear all;

%% Animation options
fig1=figure;    hold on;
bShowVectorField=true;
bRunUAV=true;
%bPlotQuiverNorm=~true;
bAnimate=true;

%% UAV parameters
r=1;

turnrate=[];
uav_v = [];
uav_v_range = [1.5 4];
velx_t = 1;
vely_t = 0;
uav_v = 5*velx_t;
%turnrate = pi;
dt = 1/(uav_v*30);

if(isempty(turnrate))
    turnrate = sqrt((uav_v+velx_t)^2/r^2);
end
if(isempty(uav_v))
    uav_v = sqrt(turnrate^2*r^2)-velx_t;
end

turnrate = turnrate*2;

%% Target initial conditions
x=0;
y=0;
xc=0;
yc=0;
theta = 0;

%% Create UAV object
xVUAV = VFUAV(dt);
xVUAV = xVUAV.SetPosition([r*-1.5 ; 0]);
uo.vx = uav_v*cos(theta);
uo.vy = uav_v*sin(theta);
uo.heading = theta;
xVUAV = xVUAV.SetVelocityAndHeading(uo);
clear uo;

%% Create instances of UAV with associated parameters
dbUAVnoNorm = xVUAV;
dbUAVnoNorm.bVFControlVelocity=~true;
dbUAVnoNorm.bVFControlHeading=~true;
dbUAVnoNorm.bDubinsPathControl=true;
dbUAVnoNorm.mTurnrate = turnrate;
dbUAVnoNorm.bNormVFVectors=~true;
dbUAVnoNorm.mLegendName = 'Summing';
% dbUAVnoNorm.mPlotColor = [0 0 1];

%% Create navigational vector fields
G=1;    %Convergence field
H=-1;   %Circulation field
L=1;    %Time-varying field

UAV_list={};
UAV_list = {UAV_list{:},dbUAVnoNorm};

cVFR = CircleVectorField('Straight',r);
cVFR.G=-G;
cVFR.H=H;
cVFR.L=L;
cVFR.xc=0;
cVFR.yc=0;
cVFR.vel_x=velx_t;
cVFR.vel_y=vely_t;
cVFR.velPathFunc = @vPath;
cVFR.bUseVRel = ~true;
cVFR.mLegendName = 'Normalizing';
cVFR.bUsePathFunc = true;
VF_List={};
VF_List={VF_List{:},cVFR};

%% Create obsticle vector fields

oVFa = CircleVectorField('Gradient',0.01);
oVFa.G=-G;
oVFa.H=0;
oVFa.L=0;
oVFa.xc=1;
oVFa.yc=0;
oVFa.vel_x=velx_t;
oVFa.vel_y=vely_t;
oVFa.velPathFunc = @vPath;
oVFa.bUsePathFunc = true;
oVFa.bUseVRel = ~true;
oVFa.mLegendName = 'Obstacle 1';

oVF1.VF = oVFa;
oVF1.plotcenter = [oVFa.xc,oVFa.yc];
oVF1.plotrange = 1.5;
oVF1.plotradius = 0.5;

oVFb = CircleVectorField('Gradient',0.01);
oVFb.G=-G*2;
oVFb.H=0;
oVFb.L=0;
oVFb.xc=4;
oVFb.yc=0;
oVFb.vel_x=velx_t;
oVFb.vel_y=vely_t;
oVFb.velPathFunc = @vPath;
oVFb.bUsePathFunc = true;
oVFb.bUseVRel = ~true;
oVFb.mLegendName = 'Obstacle 2';

oVF2.VF = oVFb;
oVF2.plotcenter = [oVFb.xc,oVFb.yc];
oVF2.plotrange = 1.5;
oVF2.plotradius = 0.5;
avoidVF = {};
avoidVF = {avoidVF{:},oVF1};
avoidVF = {avoidVF{:},oVF2};


%% Run simulation

legend_names = {'Guidance Field','Obstacle Field','Target','UAV'};

color{1} = [0 0 1];
color{2} = [1 0 0];
color{3} = [0.1 1 0.1];
t_list=0:dt:2;

plot_h_avoid=[];
RET_VF=[];
fig1.Position = [0 0 1200 800];
framecount=1;                               %Used in case the data plotted number is less than all data
for k=1:length(t_list)
    cla;
    t=t_list(k);
    if(bShowVectorField)
        clear opt;
        opt.bShowCircle=true;
        opt.bPlotQuiverNorm=true;
        opt.DecayFunc = @NoVDecay;
        opt.CustomNumberOfPoints=25;
        for ii=1:length(VF_List)
            opt.Color = color{ii};
            opt.UAV = UAV_list{ii};
            RET_VF{ii} = VF_List{ii}.PlotFieldAroundRadius(gca,opt);
            plot_h_vf(ii) = RET_VF{ii}.H;
        end
        
        opt.DecayFunc = @ VDecayLinear;%VDecay;
        opt.bPlotQuiverNorm = true;
        for ii=1:length(avoidVF)
            opt.bCustomRange = avoidVF{ii}.plotrange;
            opt.bCustomCenter = avoidVF{ii}.plotcenter;
            opt.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
            opt.Color = [1 0 0];
            opt.UAV = UAV_list{1};
            RET_temp = avoidVF{ii}.VF.PlotFieldAroundRadius(gca,opt);
            plot_h_avoid = RET_temp.H;
        end
    end
    
    if(bRunUAV)
        for ii=1:length(UAV_list)
            err{k,ii} = UAV_list{ii}.ComputePositionError(VF_List{ii});
            opt.DecayFunc = @VDecayLinear;
            opt.oVFList = avoidVF;
            
            UAV_list{ii} = UAV_list{ii}.UpdateControlFromVF(VF_List{ii},t,opt);
            if(isempty(VF_List{ii}.radFunc))
                VF_List{ii} = VF_List{ii}.UpdatePosition(t,dt);
            else
                uav_v = UAV_list{ii}.GetVelocityV();
                uavv.x = uav_v(1);
                uavv.y = uav_v(2);
                
                VF_List{ii} = VF_List{ii}.UpdatePosition(t,dt,uavv,opt);
            end
            %hold off;
            plot_vf_center = scatter(VF_List{ii}.xc_history,VF_List{ii}.yc_history,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);   
            %hold on;
        end
          
        if(bAnimate)
            for ii=1:length(UAV_list)
                xUAV = UAV_list{ii};
                plot_h_uav(ii) = xUAV.PlotPathHistory(gca,color{ii});
            end
        end
    end
    axis equal;
    grid on;
    xlabel('X-Position [-]');
    ylabel('Y-Position [-]');
    if(bAnimate)
        
        plot_handles=[];
        if(~isempty(plot_h_vf))
            plot_handles = [plot_handles,plot_h_vf];
        end
        if(~isempty(plot_h_avoid))
            plot_handles = [plot_handles,plot_h_avoid];
        end
        if(~isempty(plot_h_uav))
            plot_handles = [plot_handles,plot_h_uav];
        end
        if(~isempty(plot_vf_center))
            plot_handles = [plot_handles,plot_vf_center];
        end
        legend(plot_handles,legend_names);
        %legend([plot_h_vf,plot_h_uav,plot_vf_center],legend_names);
        %legend([plot_vf_center,plot_h_uav],);
        drawnow();
        F(framecount)=getframe(fig1);           %Save the frame
        framecount=framecount+1;
    end
end
% v=VideoWriter('VFRun.avi'); %Creates the file

%% Create video file

v=VideoWriter('VFRun.mp4','MPEG-4'); %Creates the file
% v=VideoWriter('newfile.mj2','Motion JPEG 2000');
v.FrameRate = 15;                           %Set the speed
% v.Quality = 90;
open(v);
writeVideo(v,F);

close(v);

%% Plot deviation from circumnavigation

figure;hold all;
for ii=1:length(UAV_list)
    for k=1:length(err)   
        xerr(ii,k) = err{k,ii}.dist_edge;
    end 
    plot(t_list,xerr(ii,:),'LineWidth',2,'Color',color{ii});
end
xlabel('Time [s]');
ylabel('Deviation from Circumnavigation [-]');
grid on;
legend('Summing','Normalizing','Lyapunov');

% plot(t_list,xerr(1,:));
% plot(t_list,xerr(2,:));

%% Define vehicle trajectory

function vel = vPath(t)
%     vel=[0;0];
%     return;
    v = 1;
    vel = [0,0];

end

function rad = RadFunc(vel_ratio)
    rad = vel_ratio/5;
end

%% Obsticle vector field magnitude function

function G = NoVDecay(r)
    G = ones(1,length(r));
end

function G = VDecayLinear(r)
    R = 0.5;
    G = -(1/R).*r + 1;
    nullme = find(G < 0);
    G(nullme) = zeros(1,length(nullme));
end

function G = VDecayTanh(rrin)
    rrt = -((rrin)*2*pi-pi);
    G = tanh(rrt)/2+0.5;
end
