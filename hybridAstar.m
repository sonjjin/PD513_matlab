function [img_w_path, turn_point] = hybridAstar(img_droid_cam, img_old, goalpose)
    close all
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
    img_droid_cam = imread('./images/parkinglot.png');
    [img_no_path, state] = img_cal(img_droid_cam);
%     figure(3)
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
    goalpose = [4.3*th 1.0*th pi 24];
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
            flip_map = flipud(img_no_path);
            for i = 1:size(path_front1_idx(:,1))
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),1) = 0.4940*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),2) = 0.1840*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_front2_idx(:,1))
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),1) = 0.4940*255;
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),2) = 0.1840*255;
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_back_idx(:,1))
                flip_map(path_back_idx(i,2),path_back_idx(i,1),1) = 0.6350*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),2) = 0.0780*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),3) = 0.1840*255;
            end
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning complete')
        else
            flip_map = flipud(img_old);
            for i = 1:size(path_front1_idx(:,1))
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),1) = 0.4940*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),2) = 0.1840*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_front2_idx(:,1))
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),1) = 0.4940*255;
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),2) = 0.1840*255;
                flip_map(path_front2_idx(i,2),path_front2_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_back_idx(:,1))
                flip_map(path_back_idx(i,2),path_back_idx(i,1),1) = 0.6350*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),2) = 0.0780*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),3) = 0.1840*255;
            end
            
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning error')
        end

    elseif floor(goalpose(4)/10) == 2
        [~,locs] = findpeaks(Ft, 'MinPeakDistance',s(1)-3);
        path_front1_idx = ceil(refpath.States(1:locs,1:2));
        path_back_idx = ceil(refpath.States(locs:end,1:2));
        turn_point(1,:) = refpath.States(locs,1:2);
        turn_point(3,1) = goalpose(4);
        if state == 1
            flip_map = flipud(img_no_path);
            for i = 1:size(path_front1_idx(:,1))
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),1) = 0.4940*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),2) = 0.1840*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_back_idx(:,1))
                flip_map(path_back_idx(i,2),path_back_idx(i,1),1) = 0.6350*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),2) = 0.0780*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),3) = 0.1840*255;
            end
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning complete')
        else
            flip_map = flipud(img_old);
            for i = 1:size(path_front1_idx(:,1))
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),1) = 0.4940*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),2) = 0.1840*255;
                flip_map(path_front1_idx(i,2),path_front1_idx(i,1),3) = 0.5560*255;
            end
            for i = 1:size(path_back_idx(:,1))
                flip_map(path_back_idx(i,2),path_back_idx(i,1),1) = 0.6350*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),2) = 0.0780*255;
                flip_map(path_back_idx(i,2),path_back_idx(i,1),3) = 0.1840*255;
            end
            
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning error')
        end
    else
        ref_path_ind = ceil(refpath.States(:,1:2));
        turn_point(3,1) = goalpose(4);
        if state == 1
            flip_map = flipud(img_no_path);
            for i = 1:size(ref_path_ind(:,1))
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),1) = 0.4940*255;
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),2) = 0.1840*255;
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),3) = 0.5560*255;
            end
            
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning complete')
        else
            flip_map = flipud(img_old);
            for i = 1:size(ref_path_ind(:,1))
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),1) = 0.4940*255;
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),2) = 0.1840*255;
                flip_map(ref_path_ind(i,2),ref_path_ind(i,1),3) = 0.5560*255;
            end
            
        %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
            img_w_path = flipud(flip_map);
            display('path planning error')
        end
    end

%     show fig
    f1 = figure('position',[-1080, 721, 560, 420]);
    f2 = figure('position',[-516, 720, 560, 420]);
    f3 = figure('position',[-1078, 212, 560, 420]);
    figure(f1)
    show(map)
    hold on
    figure(f2)
    show(planner)
    hold on
    figure(f3)
    imshow(img_w_path)
    
end
