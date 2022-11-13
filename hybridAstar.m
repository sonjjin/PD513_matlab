function img_w_path = hybridAstar(img_droid_cam)
%     close all
    load ./datas/map.mat
%     f1 = figure('position',[-1080, 721, 560, 420]);
%     f2 = figure('position',[-516, 720, 560, 420]);
%     f3 = figure('position',[-1078, 212, 560, 420]);
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
    
    planner = plannerHybridAStar(sv,'MinTurningRadius',1.6*th,'MotionPrimitiveLength',0.7*th);
    % planner = plannerRRTStar(validator);
    
    startPose = [4*th 3.6*th pi];
    plot(startPose(1),startPose(2),'.')
    goalPose = [0.5*th 0.5*th 0]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410
    refpath = plan(planner,startPose,goalPose);     
    % refpath = plan(planner,startPose,goalPose);
    ref_path_ind = ceil(refpath.States(:,1:2));
%     figure(f3)
%     img = imread('./images/parkinglot.png');
    img_no_path = img_cal(img_droid_cam);
    flip_map = flipud(img_no_path);
    for i = 1:size(ref_path_ind(:,1))
        flip_map(ref_path_ind(i,2),ref_path_ind(i,1),1) = 255;
        flip_map(ref_path_ind(i,2),ref_path_ind(i,1),2) = 0;
        flip_map(ref_path_ind(i,2),ref_path_ind(i,1),3) = 0;
    end
    
%     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
    img_w_path = flipud(flip_map);
    imshow(img_w_path)

%     imshow(x)
%     plot((refpath.States(:,1)),(refpath.States(:,2)),'-','Color','b')
%     [r, c] = find(Realmap);
%     plot(c,r,'.','Color','b')
    % show fig
%     figure(f1)
%     show(map)
%     hold on
%     figure(f2)
%     show(planner)
%     hold on

end
