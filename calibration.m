function img = calibration()
    % close all
    img_rgb = imread('images/birdeye_blue.jpg');
    % P = impixel(img);
    
    img_hsv = rgb2hsv(img_rgb);
    img_hsv_h = img_hsv(:,:,1);
    img_hsv_s = img_hsv(:,:,2);
    img_hsv_v = img_hsv(:,:,3);
    % imshow(img_hsv_h)
    img_hsv_blue = double(zeros(size(img_hsv_h))); 
    
    for i = 1: size(img_hsv_blue, 1)
        for j = 1:size(img_hsv_blue, 2)
            if (img_hsv_h(i, j) > 0.6 && img_hsv_h(i, j) < 0.8) && (img_hsv_v(i, j) < 1) && (img_hsv_s(i,j) > 0.4) 
                img_hsv_blue(i, j) = 1;
            end
        end
    end
    % img_clean = img_hsv_blue;
    SE = strel('square', 5);
    img_clean = imopen(img_hsv_blue, SE);
    
    img_rgb_blue = uint8(zeros([size(img_hsv_blue), 3]));
    
    BW = img_clean*255;
%     figure(1); imshow(img_clean)
    % dim = size(BW)
    [r, c] = find(BW);
    %  = min(r);
    % col = min(c);
    % % col = round(dim(2)/2)-90;
    % % row = min(find(BW));
    % boundary = bwtraceboundary(BW,[row, col],'N');
    % imshow(img_hsv_blue)
    % hold on;
    % plot(boundary(:,2),boundary(:,1),'g','LineWidth',3);
    
    
    for i = 1:size(img_hsv_blue, 1)
        for j = 1:size(img_hsv_blue, 2)
            if img_clean(i, j) ~= 0
                img_rgb_blue(i, j, :) = img_rgb(i, j, :);
            end
        end
    end
    % [r,c] = find(img_clean);
    y_lu = min(r);
    y_rl = max(r);
    x_lu = min(c);
    x_rl = max(c);
    
    lu = [x_lu+20, y_lu]';
    ll = [x_lu, y_rl]';
    rl = [x_rl, y_rl]';
    ru = [x_rl, y_lu]';
    
    x_ll = 0;
    y_ll = 0;
    x_ru = 379;
    y_ru = 223;
    target_lu = [x_ll, y_ru]';
    target_ll = [x_ll, y_ll]';
    target_rl = [x_ru, y_ll]';
    target_ru = [x_ru, y_ru]';
    
    pin = [lu, ll, rl, ru];
    target = [target_ll, target_lu, target_ru, target_rl];
%     img_tar = img_rgb(y_lu:y_rl, x_lu:x_rl);
%     imshow(img_tar)
    H = fitgeotrans(pin',target', 'projective');
    [Iwarp, reft] = imwarp(img_rgb, H);
    
%     figure(2); imshow(Iwarp)
%     figure(3); imshow(img_rgb)
%     hold on
%     plot(pin(1,:), pin(2,:), '.', 'Color', 'red')
    x = [0 0 1]*H.T;
    x = abs(int32(x));
    % x = x/x(3)
    
    img = Iwarp(target_ll(2)+x(2)-192:target_ll(2)+target_lu(2)+x(2),target_ll(1)+x(1):target_ll(1)+target_ru(1)+x(1)+9,:);
%     figure(4); imshow(img)
%     imwrite(img, 'parkinglot.png')
%     load('map2.mat');
%     [r c] = find(Realmap);
%     plot(c, r, '.', 'color', 'r');
end




