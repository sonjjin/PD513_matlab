clear
close all
clc
load './datas/angles.mat'
vel = 20;
dt = 0.01;
X = zeros(400,1);
Y = zeros(400,1);
x_t = zeros(400,1);
y_t = zeros(400,1);
z_rad_sum = 0;
% f1 = figure('position',[-1080, 721, 560, 420]);
% f2 = figure('position',[-516, 720, 560, 420]);
% f3 = figure('position',[-1078, 212, 560, 420]);
for i = 1:151
    z_rad = deg2rad(z(i));
%     if z_rad < 0.4
%         z_rad = 0;
%     end
    
    z_rad_sum = z_rad + z_rad_sum;
    [coord_glo, coord_t] = GeneralLineartransfrom(X(i),Y(i),0,vel,dt,z_rad);
    X(i+1) = X(i)+coord_t(1);
    Y(i+1) = Y(i)+coord_t(2);
    ang = atan2(coord_glo(2), coord_glo(1));
    x_t(i+1) = x_t(i) + vel*cos(ang)*dt;
    y_t(i+1) = y_t(i) + vel*sin(ang)*dt;
    figure(1)
    plot(X(i),Y(i),'.','Color',[1, i/151, 0])
    hold on
    figure(2)
    plot(i, z_rad,'.','Color',[i/151,0,1])
    hold on
    figure(3)
    plot(x_t(i),y_t(i),'.','color',[0,0.5,i/151])
    hold on
end
