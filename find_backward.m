clear
clc
close all
turn_point = [0, 0; 0, 0];
f1 = figure('position',[-1080, 721, 560, 420]);
load 'datas/ref_path_f4.mat'
goalpose = [0 0 0 14];
path = refpath.States(:,1:2);
Fx = gradient(path(:,1));
Fy = gradient(path(:,2));
[theta, rho] = cart2pol(Fx,Fy);
Ft = gradient(theta);
s = size(Ft);
if goalpose(4) == 14
    [pks,locs] = findpeaks(Ft, 'MinPeakDistance',s(1)-3);
    [pks2,locs2] = findpeaks(-Ft, 'MinPeakDistance',s(1)-3);
    turn_point(1,:) = refpath.States(locs,1:2);
        turn_point(2,:) = refpath.States(locs2,1:2);
    path_front1 = path(1:locs,1:2);
    path_back = path(locs:locs2,1:2);
    path_front2 = path(locs2:end,1:2);
    % 
    figure(f1);
    plot(path_front1(:,1), path_front1(:,2), 'color', 'r')
    hold on
    plot(path_back(:,1), path_back(:,2), 'color', 'b')
    plot(path_front2(:,1), path_front2(:,2), 'color', 'r')
    plot(turn_point(1,1), turn_point(1,2), '.', 'color', 'g', 'markersize', 15)
    plot(turn_point(2,1), turn_point(2,2), '.', 'color', 'g', 'markersize', 15)

elseif floor(goalpose(4)/10) == 2
    [pks,locs] = findpeaks(Ft, 'MinPeakDistance',s(1)-3);
    path_front1 = path(1:locs,1:2);
    path_back = path(locs:end,1:2);
    turn_point(1,:) = refpath.States(locs,1:2);
    figure(f1);
    plot(path_front1(:,1), path_front1(:,2), 'color', 'r')
    hold on
    plot(path_back(:,1), path_back(:,2), 'color', 'b')
    plot(turn_point(1,1), turn_point(1,2), '.', 'color', [0,1,0], 'MarkerSize', 15)
%     plot(turn_point(1,1), turn_point(1,2), 'color', 'g')

else
    figure(f1);
    plot(path(:,1), path(:,2), 'color', 'r')
end
