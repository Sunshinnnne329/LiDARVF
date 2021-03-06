clc;
clear all;
%close all;


shape_x = [-15 -6 8 17 10 15 -10 -8.5 -15];
shape_y = [0 7 8 12 3 -5 -7 -5 0];

r_x = 0;
r_y = 0;

robot = [r_x r_y];

obs_r = 1.25;
phi = 0:pi/12:2*pi;

obs_x = 7*rand(1,1)+obs_r*cos(phi);
obs_y = 7*rand(1,1)+obs_r*sin(phi);

% obs_x = rand(1,1).*[2 4 4 2];
% obs_y = rand(1,1).*[2 4 2 4];

% obs_x = awgn(obs_x,.0000001);
% obs_y = awgn(obs_y,.0000001);

t_x = 1:length(shape_x);
t_y = 1:length(shape_y);

t_xs = 1:1/1000:length(shape_x);
t_ys = 1:1/1000:length(shape_y);

xs = spline(t_x,shape_x,t_xs);
ys = spline(t_y,shape_y,t_ys);

% xs = awgn(xs,.00000001);
% ys = awgn(ys,.00000001);



d_wall = [];
th_wall = [];

d_obs = [];
th_obs = [];



for i = 1:length(t_xs)
    d_wall(i) = sqrt((r_x-xs(i))^2+(r_y-ys(i))^2);
    th_wall(i) = atan2((ys(i)-r_y),(xs(i)-r_x));   
end

for i = 1:length(obs_x)
    d_obs(i) = sqrt((r_x-obs_x(i))^2+(r_y-obs_y(i))^2);
    th_obs(i) = atan2((obs_y(i)-r_y),(obs_x(i)-r_x));

    
end

ind = find(max(th_obs)>th_wall & min(th_obs)<th_wall);
ind2 = find(max(th_obs)==th_obs);
ind3 = find(min(th_obs)==th_obs);
ind4 = find(d_obs(ind2)<d_obs & d_obs(ind3)<d_obs); 

xs(ind) = [];
ys(ind) = [];

obs_x_store = obs_x;
obs_y_store = obs_y;

obs_x(ind4) = [];
obs_y(ind4) = [];

room_vec_x = horzcat(xs,obs_x);
room_vec_y = horzcat(ys,obs_y);
room_vec_th_rad = atan2(room_vec_y,room_vec_x);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% THIS IS THE REPLICATED LIDAR DATA %%%%%%%%%%%%%%%%%

room_vec_d = sqrt(room_vec_x.^2+room_vec_y.^2);
room_vec_th_deg = rad2deg(room_vec_th_rad);

%%%%%%%%%%%%%%%%% THIS IS THE REPLICATED LIDAR DATA %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ind5 = find(room_vec_d == min(room_vec_d));

object_x = r_x + room_vec_d(ind5)*cosd(room_vec_th_deg(ind5));
object_y = r_y + room_vec_d(ind5)*sind(room_vec_th_deg(ind5));

hold on; grid on;
axis('equal')

line([0 15*cos(min(th_obs))],[0 15*sin(min(th_obs))])
line([0 10*cos(max(th_obs))],[0 10*sin(max(th_obs))])
line(obs_x_store,obs_y_store)
scatter(shape_x,shape_y)
scatter(xs(1:10:end),ys(1:10:end),'r')
plot(shape_x,shape_y,'g')
scatter(r_x,r_y,'k','filled')
scatter(obs_x,obs_y, 'b')
scatter(object_x,object_y,'m','filled')