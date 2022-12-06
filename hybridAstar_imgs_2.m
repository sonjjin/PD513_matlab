function [img_w_path1, img_w_path2, img_w_path3, turn_point] = hybridAstar(img_droid_cam, img_old, goalpose)
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
    
    planner1 = plannerHybridAStar(sv,'MinTurningRadius',1.7*th,'MotionPrimitiveLength',0.8*th);
    planner2 = plannerHybridAStar(sv,'MinTurningRadius',1.7*th,'MotionPrimitiveLength',0.8*th);
    planner3 = plannerHybridAStar(sv,'MinTurningRadius',1.7*th,'MotionPrimitiveLength',0.8*th);
    % planner = plannerRRTStar(validator);
    
    startpose1 = [4*th 3.9*th pi];
    img_droid_cam = imread('./images/parkinglot.png');
    [img_no_path, state] = img_cal(img_droid_cam);
    [w, h, ~] = size(Realmap);
    img_w_path1_temp = zeros([w, h, 3],'uint8');
    img_w_path2_temp = zeros([w, h, 3],'uint8');
    img_w_path3_temp = zeros([w, h, 3],'uint8');
    size(img_w_path1_temp)
    img_w_path1 = img_w_path1_temp;
    img_w_path2 = img_w_path2_temp;
    img_w_path3 = img_w_path3_temp;
%     figure(5)
%     imshow(img_no_path)
%     plot(startPose(1),startPose(2),'.')
%% goal position for simulation
% 
%     goalpose = [0.4*th 2.3*th pi 11]; 
%     goalpose = [0.4*th 1.4*th pi 12]; 
%     goalPose = [0.2*th 0.1*th pi 13];
%     goalpose = [4.0*th 1.2*th 0 14];
%     goalpose = [4.4*th 0.1*th 0 15];
% 
%     goalpose = [0.4*th 2.3*th 0 21];
%     goalpose = [0.4*th 1.4*th 0 22];
%     goalpose = [0.3*th 0.07*th 0 23];
    goalpose = [4.3*th 1.0*th pi 24];
%     goalpose = [4.4*th 0.1*th pi 25];
%     


    if goalpose(4) == 14
        goalpose1 = [235.544106853132, 175.467909679650, -1.21202090315039];
        goalpose2 = [222.768723405011, 202.187707775615, -1.03758303757109];
        goalpose3 = goalpose(1:3);
        startpose2 = goalpose1;
        startpose3 = goalpose2;

        refpath1 = plan(planner1,startpose1,goalpose1);
        refpath2 = plan(planner2,startpose2,goalpose2);
        refpath3 = plan(planner3,startpose3,goalpose3);

    elseif floor(goalpose(4)/10) == 2
        if goalpose(4) == 21
            goalpose1 = [193.750360052467, 132.531634758170, -1.13088618147206];
        elseif goalpose(4) == 22
            goalpose1 = [178.004796058040, 69.2707220935693, -0.947215947046598];
        elseif goalpose(4) == 23
            goalpose1 = [224.172427493734, 11.0118952691825, -0.458958482908784];
        elseif goalpose(4) == 24
            goalpose1 = [130, 250, pi];
        elseif goalpose(4) == 25
            goalpose1 = [164.010324899045, 20.2688145470035, -2.47828741061746];
        end

        goalpose2  = goalpose(1:3);
        startpose2 = goalpose1;

        refpath1 = plan(planner1,startpose1,goalpose1);
        refpath2 = plan(planner2,startpose2,goalpose2);

    else
        goalpose1  = goalpose(1:3);
        refpath1 = plan(planner1,startpose1,goalpose1);
    end



    if goalpose(4) == 14
        path1_idx = ceil(refpath1.States(:,1:2));
        path2_idx = ceil(refpath2.States(:,1:2));
        path3_idx = ceil(refpath3.States(:,1:2));
        turn_point(1,:) = startpose2(1:2);
        turn_point(2,:) = startpose3(1:2);
        turn_point(3,1) = goalpose(4);
        if state == 1
            for i = 1:size(path1_idx(:,1))
                img_w_path1_temp(path1_idx(i,2),path1_idx(i,1),:) = 255;
            end
            for i = 1:size(path3_idx(:,1))
                img_w_path2_temp(path3_idx(i,2),path3_idx(i,1),:) = 255;
            end
            for i = 1:size(path2_idx(:,1))
                img_w_path3_temp(path2_idx(i,2),path2_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            img_w_path3 = flipud(img_w_path3_temp);
            display('path planning complete')
        else
            for i = 1:size(path1_idx(:,1))
                img_w_path1_temp(path1_idx(i,2),path1_idx(i,1),:) = 255;
            end
            for i = 1:size(path2_idx(:,1))
                img_w_path2_temp(path2_idx(i,2),path2_idx(i,1),:) = 255;
            end
            for i = 1:size(path3_idx(:,1))
                img_w_path3_temp(path3_idx(i,2),path3_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            img_w_path3 = flipud(img_w_path3_temp);
            display('path planning error')
        end

    elseif floor(goalpose(4)/10) == 2
        path1_idx = ceil(refpath1.States(:,1:2));
        path2_idx = ceil(refpath2.States(:,1:2));
        turn_point(1,:) = startpose2(1:2);
        turn_point(3,1) = goalpose(4);
        if state == 1
            for i = 1:size(path1_idx(:,1))
                img_w_path1_temp(path1_idx(i,2),path1_idx(i,1),:) = 255;
            end
            for i = 1:size(path2_idx(:,1))
                img_w_path2_temp(path2_idx(i,2),path2_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            display('path planning complete')
        else
            for i = 1:size(path1_idx(:,1))
                img_w_path1_temp(path1_idx(i,2),path1_idx(i,1),:) = 255;
            end
            for i = 1:size(path2_idx(:,1))
                img_w_path2_temp(path2_idx(i,2),path2_idx(i,1),:) = 255;
            end
            img_w_path1 = flipud(img_w_path1_temp);
            img_w_path2 = flipud(img_w_path2_temp);
            display('path planning error')
        end
    
    else
        ref_path_ind = ceil(refpath1.States(:,1:2));
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
    f1 = figure('position',[-1080, 721, 560, 420]);
    f2 = figure('position',[-516, 720, 560, 420]);
    f3 = figure('position',[-1078, 212, 560, 420]);
    f4 = figure('position',[-516, 212, 560, 420]);
    f5 = figure('position',[-1078, -298, 560, 420]);
    f6 = figure('position',[-516, -298, 560, 420]);
    figure(f1)
    show(map)
%     turn_point
    hold on
    figure(f2)
    show(planner1)
    hold on
    figure(f3)
    show(planner2)

    figure(f4)
    imshow(img_w_path1)
    hold on
    figure(f5);
    imshow(img_w_path2)
    figure(f6);
    imshow(img_w_path3)
%     

end
