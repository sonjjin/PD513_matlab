function [img, state] = img_cal(img_droid_cam)
%     close all
%     img_droid_cam = imread('./images/parkinglot.png');
%     figure(1)
%     imshow(img_droid_cam)
    state = 0;
    img_hsv = rgb2hsv(img_droid_cam);
    img_hsv_h = img_hsv(:,:,1);
    img_hsv_s = img_hsv(:,:,2);
    img_hsv_v = img_hsv(:,:,3);
    img_hsv_green = double(zeros(size(img_hsv_h))); 
    
    for i = 1: size(img_hsv_green, 1)
        for j = 1:size(img_hsv_green, 2)
            if (img_hsv_h(i, j) > 0.23 && img_hsv_h(i, j) < 0.54) && (img_hsv_v(i, j) < 1) && (img_hsv_s(i,j) > 0.4) 
                img_hsv_green(i, j) = 1;
            end
        end
    end
%     img_clean = img_hsv_green;
    SE = strel('square', 10);
%     SE = strel('square', 4);
    img_clean = imopen(img_hsv_green, SE);
    img_clean = imdilate(img_clean, SE);
    img_clean = imbinarize(img_clean);
%     SE = strel('disk',15);
%     img_clean = imdilate(img_clean,SE);
%     f1 = figure('position',[-871 597 814 573]);
%     figure(3)
    imshow(img_clean)
%     hold on
    
    BW = img_clean;
    [r, c] = find(BW);
    stats = regionprops('table',BW,'Centroid');
    centers = round(stats.Centroid);
%     plot(centers(1,1)-5, centers(1,2)+5, '.', 'Color', 'r')
    check = size(centers)
    
    if check(1) == 4
        ll = [centers(1,1)-5, centers(1,2)+5]';
        lu = [centers(2,1)-5, centers(2,2)+5]';
        ru = [centers(3,1)+10, centers(3,2)+5]';
        rl = [centers(4,1)+10, centers(4,2)+5]';
        
        comp = 10;
        x_ll = 0;
        y_ll = 0;
        x_ru = 465 - comp;
        y_ru = 186 - comp;
        target_lu = [x_ll, y_ru]';
        target_ll = [x_ll, y_ll]';
        target_rl = [x_ru, y_ll]';
        target_ru = [x_ru, y_ru]';
        
        pin = [lu, ll, rl, ru];
        target = [target_ll, target_lu, target_ru, target_rl];
        H = fitgeotrans(pin',target', 'projective');
        [Iwarp, reft] = imwarp(img_droid_cam, H);
        
        x = [0 0 1]*H.T;
        x = abs(int32(x));
        % x = x/x(3)
        state = 1;
        img = Iwarp(target_ll(2)+x(2)-256-comp:target_ll(2)+target_lu(2)+x(2),target_ll(1)+x(1)-comp:target_ll(1)+target_ru(1)+x(1)-1,:);
        display('calibration complete')
    else
        img = 0;
        display('calibration error')
    end


 %     f2 = figure('position',[-625 -172 627 751]);
%     figure(f2)
%     imshow(img)
%     imwrite(img, 'parkinglot.png')
%     load('map2.mat');
%     [r c] = find(Realmap);
%     plot(c, r, '.', 'color', 'r');
end




