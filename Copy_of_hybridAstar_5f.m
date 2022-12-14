function img_w_path = hybridAstar(img_droid_cam, img_old)
%     close all
    load ./datas/map.mat
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
    
    startPose = [4*th 3.8*th pi];
    figure(3)
    plot(startPose(1),startPose(2),'.')
    goalPose = [0.5*th 0.5*th pi]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410
    refpath = plan(planner,startPose,goalPose);     
    % refpath = plan(planner,startPose,goalPose);
    ref_path_ind = ceil(refpath.States(:,1:2));
%     figure(f3)
%     img_droid_cam = imread('./images/parkinglot.png');

   imshow(img_droid_cam)
    [img_no_path, state] = img_cal(img_droid_cam);
    imshow(img_no_path)
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
    figure(2)
    imshow(img_w_path)


%     img_no_path = img_cal(img_droid_cam);
%     flip_map = flipud(img_no_path);
%     for i = 1:size(ref_path_ind(:,1))
%         flip_map(ref_path_ind(i,2),ref_path_ind(i,1),1) = 0.4940*255;
%         flip_map(ref_path_ind(i,2),ref_path_ind(i,1),2) = 0.1840*255;
%         flip_map(ref_path_ind(i,2),ref_path_ind(i,1),3) = 0.5560*255;
%     end
%     
% %     x = flip_map(ref_path_ind(:,2),ref_path_ind(:,1),:);
%     img_w_path = flipud(flip_map);
%     imshow(img_w_path)
%     imwrite(img_w_path,'path.png')
% >>>>>>> 5be582cce9a3acaa6b77c54d14b345cbb71160d4
%     imshow(x)
%     plot((refpath.States(:,1)),(refpath.States(:,2)),'-','Color','b')
%     [r, c] = find(Realmap);
%     plot(c,r,'.','Color','b')
%     show fig
%     f1 = figure('position',[-1080, 721, 560, 420]);
%     f2 = figure('position',[-516, 720, 560, 420]);
%     f3 = figure('position',[-1078, 212, 560, 420]);
%     figure(f1)
%     show(map)
%     hold on
%     figure(f2)
%     show(planner)
%     hold on

end
