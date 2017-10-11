clc;
clear all;
close all;

shape_x = 3.*[-15 -6 8 17 10 15 -10 -8.5 -15];
shape_y = 3.*[0 7 8 12 3 -10 -7 -5 0];

r_x = 0;
r_y = 0;

xs = [];
ys = [];

for i = 1:length(shape_x)-1
    x = linspace(shape_x(i),shape_x(i+1),1000);
    y = linspace(shape_y(i),shape_y(i+1),1000);
    
    xs = horzcat(xs,x);
    ys = horzcat(ys,y);
end

xs = awgn(xs,3);
ys = awgn(ys,3);



obs_r = 1.25;
phi = 0:pi/12:2*pi;

obs_x = 20*rand(1,1)+obs_r*cos(phi);
obs_y = 20*rand(1,1)+obs_r*sin(phi);

% obs_x = rand(1,1).*[2 4 4 2];
% obs_y = rand(1,1).*[2 4 2 4];
 
% obs_x = 4+obs_r*cos(phi);
% obs_y = 4+obs_r*sin(phi);

% obs_x = awgn(obs_x,10);
% obs_y = awgn(obs_y,10);



d_wall = [];
th_wall = [];

d_obs = [];
th_obs = [];



for i = 1:length(xs)
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

% Round 1 of Clustering
idx = kmeans(room_vec_d',40);
dist_array = [room_vec_d', idx];
cluster_ind = find(min(room_vec_d)==room_vec_d);
idx_store = find(idx==idx(cluster_ind));

x_ave = 0;
y_ave = 0;

for i = 1:length(idx_store)
    x_ave = x_ave + room_vec_d(idx_store(i))*cosd(room_vec_th_deg(idx_store(i)));
    y_ave = y_ave + room_vec_d(idx_store(i))*sind(room_vec_th_deg(idx_store(i)));
end

x_ave = x_ave/length(idx_store);
y_ave = y_ave/length(idx_store);

figure;
hold on; grid on;
axis('equal')

line([0 15*cos(min(th_obs))],[0 15*sin(min(th_obs))])
line([0 10*cos(max(th_obs))],[0 10*sin(max(th_obs))])
line(obs_x_store,obs_y_store)
scatter(shape_x,shape_y)
scatter(room_vec_x,room_vec_y,[],dist_array(:,2))
colorbar
plot(shape_x,shape_y,'g')
scatter(r_x,r_y,'k','filled')
scatter(room_vec_x(idx_store),room_vec_y(idx_store),'m','filled')
scatter(x_ave,y_ave,50,'g','filled')
% scatter(room_vec_x(idx_store2),room_vec_y(idx_store2),'g','filled')
