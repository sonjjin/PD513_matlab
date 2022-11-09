function [mask, rr, rc, br, bc] = color_extractor(img)
%     clear
%     clf
%     
%     f = figure(1);
    % % set(f, 'OuterPosition', [-1078 315 1078 873])
%     f.Position = [-1078 315 1078 873];
%     hold on
%     img = imread('color_test.jpg');
    [rmask, rr, rc] = color_detection(img, 1);
    [bmask, br, bc] = color_detection(img, 2);
    mask = rmask + bmask;
%     imshow(mask)
%     f.Position = [-1078 315 1078 873];
%     hold on
%     imshow(bmask)
%     plot(rr, rc, 'o', 'MarkerEdgeColor', 'red')
%     hold on
%     plot(br, bc, 'o', 'MarkerEdgeColor', 'blue')
    
    % f = figure('units', 'pixels', 'pos', [-1078 315 1078 873]);
    
    
    
%     f.Position = [-1078 315 1078 873];
%     [rr,rc] = find(img4);
%     rr_max = max(rr);
%     rr_min = min(rr);
%     rc_max = max(rc);
%     rc_min = min(rc);
%     
%     rr_m = (rr_max+rr_min)/2;
%     rc_m = (rc_max+rc_min)/2;
%     figure(1)
%     

end

function [mask, row, col] = color_detection(img, mode)
    img_hsv = rgb2hsv(img);
    img_hsv(:,:,1) = img_hsv(:,:,1)*180;
    img_hsv(:,:,2) = img_hsv(:,:,2)*255;
    [R, C, X] = size(img); 
    temp = zeros([R,C,X]);
    mask = zeros([R, C, 1]);
    se = strel('disk',10);
    if mode == 1
        % find red 
        for i = 1:R
            for j = 1:C
                if img(i,j,1) - img(i,j,2) < 60 || img(i,j,1) - img(i,j,3) < 60
        %             img2(i,j,1) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
        %             img2(i,j,2) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
        %             img2(i,j,3) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
                    mask(i,j,1) = 0;
                    mask(i,j,2) = 0;
                    mask(i,j,3) = 0;
                else
                    mask(i,j,:) = 1;
                end
            end
        end
    elseif mode == 2
        % find blue
        for i = 1:R
            for j = 1:C
                if img(i,j,3) - img(i,j,2) < 50 || img(i,j,3) - img(i,j,1) < 50
        %             img2(i,j,1) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
        %             img2(i,j,2) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
        %             img2(i,j,3) = (img(i,j,1)+img(i,j,2)+img(i,j,3))/3;
                    mask(i,j,1) = 0;
                    mask(i,j,2) = 0;
                    mask(i,j,3) = 0;
                else
                    mask(i,j,:) = 1;
                end
            end
        end
    end
    mask = rgb2gray(mask);
    mask = imopen(mask,se);
    [rownz,colnz] = find(mask);
    
    r_max = max(rownz);
    r_min = min(rownz);
    c_max = max(colnz);
    c_min = min(colnz);
    col = (r_max+r_min)/2;
    row = (c_max+c_min)/2;
end





