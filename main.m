clear
clc
clf

m = mobiledev;
cam = camera(m,'back');
cam.Autofocus = 'on';
figure(1)
hold on
time_ = 0;
tic
while(1)
    img = snapshot(cam, 'immediate');
    [mask, rr, rc, br, bc] = color_extractor(img);
    imshow(img)
    hold on
    plot([rr,br], [rc,bc],'-')
    toc
    pause(5)
    
end
