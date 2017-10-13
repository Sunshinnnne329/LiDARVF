clc;
clear all;
close all;

shape_x = 3.*[-15 -15 15 15 -15];
shape_y = 3.*[-15 15 15 -15 -15];

xs = [];
ys = [];

for i = 1:length(shape_x)-1
    x = linspace(shape_x(i),shape_x(i+1),1000);
    y = linspace(shape_y(i),shape_y(i+1),1000);
    
    xs = horzcat(xs,x);
    ys = horzcat(ys,y);
end

xs_orig = awgn(xs,3);
ys_orig = awgn(ys,3);

obs_r = 1.25;
phi = 0:pi/12:2*pi;
obs_x_orig = 20*rand(1,1)+obs_r*cos(phi);
obs_y_orig = 20*rand(1,1)+obs_r*sin(phi);

r_rad = 15;
r_alp = 0:pi/12:2*pi;
r_x = r_rad.*cos(r_alp);
r_y = r_rad.*sin(r_alp);

% r_x = (0:1:15);
% r_y = (0:1:15);

figure;

for p = 1:length(r_x)
    
    xs = xs_orig;
    ys = ys_orig;
    
    obs_x = obs_x_orig;
    obs_y = obs_y_orig;
    
%     obs_x = awgn(obs_x,10);
%     obs_y = awgn(obs_y,10);

    
    d_wall = [];
    th_wall = [];
    
    d_obs = [];
    th_obs = [];
    
    for i = 1:length(xs)
        d_wall(i) = sqrt((r_x(p)-xs(i))^2+(r_y(p)-ys(i))^2);
        th_wall(i) = atan2((ys(i)-r_y(p)),(xs(i)-r_x(p)));
    end
    
    for i = 1:length(obs_x)
        d_obs(i) = sqrt((r_x(p)-obs_x(i))^2+(r_y(p)-obs_y(i))^2);
        th_obs(i) = atan2((obs_y(i)-r_y(p)),(obs_x(i)-r_x(p)));
    end
    
    ind = find(max(th_obs)>th_wall & min(th_obs)<th_wall);
    ind2 = find(max(th_obs)==th_obs);
    
    if length(ind2)>1
        ind2 = min(ind3);
    end
    
    ind3 = find(min(th_obs)==th_obs);
    if length(ind3)>1
        ind3 = min(ind3);
    end
    
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
    
    x_indexed = room_vec_d(idx_store).*cosd(room_vec_th_deg(idx_store));
    y_indexed = room_vec_d(idx_store).*sind(room_vec_th_deg(idx_store));
    
    [bool,x_circ,y_circ,R,xc,yc] = checkCircFit(x_indexed,y_indexed);
    
    if bool ~= 1
        disp("NO CIRCLE FIT FOUND. DISPLAYING DATA.");
    end
    
    x_ave = xc;
    y_ave = yc;
    
    
    hold on; grid on;
    axis('equal')
    
    l1 = line([r_x(p) r_x(p)+50*cos(min(th_obs))],[r_y(p) r_y(p)+50*sin(min(th_obs))]);
    l2 = line([r_x(p) r_x(p)+50*cos(max(th_obs))],[r_y(p) r_y(p)+50*sin(max(th_obs))]);
    line(obs_x_store,obs_y_store)
    scatter(shape_x,shape_y)
    a = scatter(room_vec_x,room_vec_y,[],dist_array(:,2));
    colorbar
    plot(shape_x,shape_y,'g')
    scatter(r_x(p),r_y(p),'p','k','filled')
    cent = scatter(room_vec_x(idx_store),room_vec_y(idx_store),'m','filled');
    scatter(x_ave,y_ave,50,'g','filled')
    plot(x_circ,y_circ)
    
    disp("Location of Object Centroid, x = "+x_ave+", y = "+y_ave);
    
    drawnow();
    
    if p ~= length(r_x)
        delete(l1);
        delete(l2);
        delete(a);
        delete(cent);
    end
end
