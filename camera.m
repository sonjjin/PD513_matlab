clear
close
clc

rosshutdown;
rosinit('http://192.168.0.85:11311/');

sub_camf = rossubscriber('/front_cam/image_raw','DataFormat','struct');
sub_caml = rossubscriber('/left_cam/image_raw','DataFormat','struct');
sub_camr = rossubscriber('/right_cam/image_raw','DataFormat','struct');
sub_camb = rossubscriber('/rear_cam/image_raw','DataFormat','struct');
while 1
    img_f_msg = receive(sub_camf);
    img_l_msg = receive(sub_caml);
    img_r_msg = receive(sub_camr);
    img_b_msg = receive(sub_camb);
    img_f = rosReadImage(img_f_msg);
    img_l = rosReadImage(img_l_msg);
    img_r = rosReadImage(img_r_msg);
    img_b = rosReadImage(img_b_msg);
    figure(1)
    imshow(img_f)
    figure(2)
    imshow(img_l)
    figure(3)
    imshow(img_r)
    figure(4)
    imshow(img_b)
end
% 
% 
% droid_cam_msg = receive(droid_cam_sub, 3);
% img_droid_ori = rosReadImage(droid_cam_msg);
% img_parkinglot = img_cal(img_droid_cam);