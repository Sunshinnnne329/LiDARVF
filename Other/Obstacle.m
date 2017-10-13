t_list=0:dt:5.5;

fig1=figure;    hold on;
bShowVectorField=true;
G=1;    %Convergence field
H=-1;   %Circulation field
L=1;    %Time-varying field
velx_t = 1;
vely_t = 0;

oVFa = CircleVectorField('Gradient',0.01);
oVFa.G=-G;
oVFa.H=0;
oVFa.L=0;
oVFa.xc=5;
oVFa.yc=0;
oVFa.vel_x=velx_t;
oVFa.vel_y=vely_t;
oVFa.velPathFunc = @vPath;
oVFa.bUsePathFunc = true;
oVFa.bUseVRel = ~true;
oVF1.VF = oVFa;
oVF1.plotcenter = [oVFa.xc,oVFa.yc];
oVF1.plotrange = 1.5;
oVF1.plotradius = 0.5;

avoidVF = {};
avoidVF = {avoidVF{:},oVF1};

for k=1:length(t_list)
    cla;
    t=t_list(k);
    if(bShowVectorField)
        clear opt;
        opt.bShowCircle=true;
        opt.bPlotQuiverNorm=true;
        opt.DecayFunc = @VDecayTanh;
        opt.CustomNumberOfPoints=25;
%         for ii=1:length(VF_List)
%             opt.Color = color{ii};
%             opt.UAV = UAV_list{ii};
%             RET_VF{ii} = VF_List{ii}.PlotFieldAroundRadius(gca,opt);
%             plot_h_vf(ii) = RET_VF{ii}.H;
%         end
        
        opt.DecayFunc = @ VDecayTanh;%VDecay;
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
    
    
          


    axis equal;
    grid on;
    xlabel('X-Position [-]');
    ylabel('Y-Position [-]');
    if(bAnimate)
        
        plot_handles=[];
        if(~isempty(plot_h_vf))
            %plot_handles = [plot_handles,plot_h_vf];
        end
        if(~isempty(plot_h_avoid))
            plot_handles = [plot_handles,plot_h_avoid];
        end
        if(~isempty(plot_h_uav))
            %plot_handles = [plot_handles,plot_h_uav];
        end
        if(~isempty(plot_vf_center))
            %plot_handles = [plot_handles,plot_vf_center];
        end
        %legend(plot_handles,legend_names);
        %legend([plot_h_vf,plot_h_uav,plot_vf_center],legend_names);
        %legend([plot_vf_center,plot_h_uav],);
        drawnow();
        %F(framecount)=getframe(fig1);           %Save the frame
        %framecount=framecount+1;
    end
end

function vel = vPath(t)
%     vel=[0;0];
%     return;
    v = 1;
    if(t<=1)
        theta = [0,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<2)
        theta = [1,0];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<3)
        theta = [1,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<4)
        theta = [0,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    else
        vel = [0;0];
    end

end

function rad = RadFunc(vel_ratio)
    rad = vel_ratio/5;
end

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