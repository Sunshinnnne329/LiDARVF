close all;
clear variables;
r=1;

z=0;
G=1;
H=1;

limit = 2;
step = 0.1;
x_list = -limit:step:limit;
y_list = -limit:step:limit;

alpha1=@(x,y,r) x^2+y^2-r^2;
alpha2=@(z) z;
Vconv=@(x,y,z,r) -G.*(alpha1(x,y,r).*[2.*x;2.*y;0]+alpha2(z).*[0;0;1]);
Vcirc=@(x,y) H.*[2.*y;-2.*x;0];
VF=@(x,y,z,r) Vconv(x,y,z,r)+Vcirc(x,y);
for i=1:length(x_list)
    x=x_list(i);
    for ii=1:length(y_list)
        y=y_list(ii);
        %alpha1 = x^2+y^2-r^2;
        %alpha2 = z;
        %Vconv{i,ii} = -G.*(alpha1(x,y,r).*[2.*x;2.*y;0]+alpha2(z).*[0;0;1]);
        %Vcirc{i,ii} = H.*[2.*y;-2.*x;0]; %page 81
        %VF_list{i,ii}=Vconv{i,ii}+Vcirc{i,ii};
        VF_list{i,ii}=VF(x,y,z,r);
        u(i,ii)=VF_list{i,ii}(1);
        v(i,ii)=VF_list{i,ii}(2);
        x_q(i,ii)=x;
        y_q(i,ii)=y;
    end
end

%[xmg,ymg] = meshgrid(x_list,y_list);
figure;
hold on;
un=u./sqrt(u.^2+v.^2);
vn=v./sqrt(u.^2+v.^2);
quiver(x_q,y_q,un,vn);
th = 0:pi/50:2*pi;
x_c=0;
y_c=0;
xunit = r * cos(th) + x_c;
yunit = r * sin(th) + y_c;
h = plot(xunit, yunit);
axis equal;
%return;
uav_x=-1.5%0.10531%-1.5;
uav_y=0%0.99469%0;
uav_v=0.1;
%uav_vx = 0.1;
%uav_vy = 0.4;
theta = 0.005;%atan2(uav_vy,uav_vx);
dt = 0.1;
t_list=0:dt:15;
turn_thres=pi/15;
for i=1:length(t_list)
    %t = t_list(i);
    
    scatter(uav_x,uav_y);
    
    %theta = atan2(uav_vx,uav_vy);
    uav_vx = uav_v*cos(theta);
    uav_vy = uav_v*sin(theta);
    %needs to be used, constrains theta to +/- pi
    theta = atan2(uav_vy,uav_vx); %update AFTER the new vel (x,y)
    uav_x = uav_x+uav_vx*dt;
    uav_y = uav_y+uav_vy*dt;

    turn_vf = VF(uav_x,uav_y,0,r);
    turn_angle = atan2(turn_vf(2),turn_vf(1));

    turnrate=turn_thres;
    beta = turn_angle;
    fprintf('(x=%4.2f,y=%4.2f) -> VF %4.2f T %4.2f Diff %4.2f',uav_x,uav_y,turn_angle,theta,theta-turn_angle);

        if abs(theta - beta) < pi
            if theta - beta < 0
                theta = theta + turnrate*dt;
                %if theta > 2*pi
                %    theta = theta - 2*pi; 
                %end
            else
                theta = theta - turnrate*dt;
                %if theta < 0
                %    theta = theta + 2*pi; 
                %end
            end
        else
            if theta - beta > 0
                theta = theta + turnrate*dt;
                %if theta > 2*pi
                %    theta = theta - 2*pi; 
                %end
            else
                theta = theta - turnrate*dt;
                %if theta < 0
                %    theta = theta + 2*pi; 
                %end
            end
        end
        if(theta >= 2*pi)
            %disp('\n!!!!!!more than 2pi!!!!!!!!!\n');
            theta = theta - (2*pi);
        elseif(theta <= -2*pi)
            theta = theta + (2*pi);
        end
    %fprintf('(x=%f,y=%f) -> VF = %f @ theta = %f\n',uav_x,uav_y,turn_angle,theta);
    fprintf(' New Theta %f\n',theta);
    drawnow();
end
