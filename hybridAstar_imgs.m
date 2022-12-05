function [img_w_path1, img_w_path2, img_w_path3, turn_point] = hybridAstar(img_droid_cam, img_old, goalpose)
%     close all
    load ./datas/map.mat
    turn_point = [0, 0; 0, 0; 0, 0];
    % mesh(Realmap)
    % img = calibration();
    th = 100;
    map = binaryOccupancyMap(Realmap,1);
    ss = stateSpaceSE2;
    ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
    sv = validatorOccupancyMap(ss);
    sv.Map = map;
    % load parkingLotCostVal.mat % costVal
    % map = binaryOccupancyMap(costVal);
    
    planner = plannerHybridAStar(sv,'MinTurningRadius',1.7*th,'MotionPrimitiveLength',0.8*th);
    % planner = plannerRRTStar(validator);
    
    startPose = [4*th 3.9*th pi];
%     img_droid_cam = imread('./images/parkinglot.png');
    [img_no_path, state] = img_cal(img_droid_cam);
    [w, h, ~] = size(Realmap);
    img_w_path1_temp = zeros([w, h, 3],'uint8');
    img_w_path2_temp = zeros([w, h, 3],'uint8');
    img_w_path3_temp = zeros([w, h, 3],'uint8');
    size(img_w_path1_temp)
    img_w_path1 = img_w_path1_temp;
    img_w_path2 = img_w_path2_temp;
    img_w_path3 = img_w_path3_temp;
    figure(5)
    imshow(img_no_path)
%     plot(startPose(1),startPose(2),'.')
%% goal position for simulation
% forward
%     goalpose = [0.4*th 2.3*th pi 11]; 
%     goalpose = [0.4*th 1.4*th pi 12]; 
%     goalpose = [0.4*th 0.1*th pi 13]; 
%     goalpose = [4.0*th 1.2*th 0 14];
%     goalpose = [4.4*th 0.1*th 0 15];

%     goalpose = [0.4*th 2.3*th 0 21];
%     goalpose = [0.4*th 1.4*th 0 22];
%     goalpose = [0.4*th 0.1*th 0 23];
%     goalpose = [4.3*th 1.0*th pi 24];
%     goalpose = [4.4*th 0.1*th pi 25];
    goalPose = goalpose(1:3);
    refpath = plan(planner,startPose,goalPose);     
    % refpath = plan(planner,startPose,goalPose);

    path = refpath.States(:,1:2);
    Fx = gradient(path(:,1));
    Fy = gradient(path(:,2));
    [theta, ~] = cart2pol(Fx,Fy);
    Ft = gradient(theta);
    s = size(Ft);
    if goalpose(4) == 14
        display('x')
        [~,locs] = findpeaks(Ft, 'MinPeakDistance',s(1)-3);
        [~,locs2] = findpeaks(-Ft, 'MinPeakDistance',s(1)-3);
        
        path_front1_idx = ceil(refpath.States(1:locs,1:2));
        path_back_idx = ceil(refpath.States(locs:locs2,1:2));
        path_front2_idx = ceil(refpath.States(locs2:end,1:2));
        turn_point(1,:) = refpath.States(locs,1:2);
        turn_point(2,:) = refpath.States(locs2,1:2);
        turn_point(3,1) = goalpose(4);
        if state == 1
            for i = 1:size(path_front1_idx(:,1))
                img_w_path1_temp(path_front1_idx(i,2),path_front1_idx(i,1),:) = 255;
            end
            for i = 1:size(path_front2_idx(:,1))
                img_w_path2_temp(path_front2_idx(i,2),path_front2_idx(i,1),:) = 255;
            end
            for i = 1:size(path_back_idx(:,1))
                img_w_path3_temp(path_back_idx(i,2),path_back_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            img_w_path3 = flipud(img_w_path3_temp);
            display('path planning complete')
        else
            for i = 1:size(path_front1_idx(:,1))
                img_w_path1_temp(path_front1_idx(i,2),path_front1_idx(i,1),:) = 255;
            end
            for i = 1:size(path_front2_idx(:,1))
                img_w_path2_temp(path_front2_idx(i,2),path_front2_idx(i,1),:) = 255;
            end
            for i = 1:size(path_back_idx(:,1))
                img_w_path3_temp(path_back_idx(i,2),path_back_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            img_w_path3 = flipud(img_w_path3_temp);
            display('path planning error')
        end

    elseif floor(goalpose(4)/10) == 2
        [~,locs] = findpeaks(Ft, 'MinPeakDistance',s(1)-3);
        path_front1_idx = ceil(refpath.States(1:locs,1:2));
        path_back_idx = ceil(refpath.States(locs:end,1:2));
        turn_point(1,:) = refpath.States(locs,1:2);
        turn_point(3,1) = goalpose(4);
        if state == 1
            for i = 1:size(path_front1_idx(:,1))
                img_w_path1_temp(path_front1_idx(i,2),path_front1_idx(i,1),:) = 255;
            end
            for i = 1:size(path_back_idx(:,1))
                img_w_path2_temp(path_back_idx(i,2),path_back_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            display('path planning complete')
        else
            for i = 1:size(path_front1_idx(:,1))
                img_w_path1_temp(path_front1_idx(i,2),path_front1_idx(i,1),:) = 255;
            end
            for i = 1:size(path_back_idx(:,1))
                img_w_path2_temp(path_back_idx(i,2),path_back_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            display('path planning error')
        end
    else
        ref_path_ind = ceil(refpath.States(:,1:2));
        turn_point(3,1) = goalpose(4);
        if state == 1
            for i = 1:size(ref_path_ind(:,1))
                img_w_path1_temp(ref_path_ind(i,2),ref_path_ind(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            display('path planning complete')
        else
            for i = 1:size(ref_path_ind(:,1))
                img_w_path1_temp(ref_path_ind(i,2),ref_path_ind(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            display('path planning error')
        end
    end

%%     show fig
%     f1 = figure('position',[-1080, 721, 560, 420]);
%     f2 = figure('position',[-516, 720, 560, 420]);
%     f3 = figure('position',[-1078, 212, 560, 420]);
%     figure(f1)
%     show(map)
%     turn_point
%     hold on
%     figure(f2)
%     show(planner)
%     hold on
%     figure(f3)
%     imshow(img_w_path1)
%     hold on
%     figure;
%     imshow(img_w_path2)
%     figure;
%     imshow(img_w_path3)
%     

end
