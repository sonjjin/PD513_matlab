clear
close all
clc

rosshutdown;
rosinit('http://192.168.0.85:11311/');
% rosinit
%% publisher
[pub_img_w_path1, msg_img_w_path1] = rospublisher('/img_w_path1','sensor_msgs/Image');
[pub_img_w_path2, msg_img_w_path2] = rospublisher('/img_w_path2','sensor_msgs/Image');
[pub_img_w_path3, msg_img_w_path3] = rospublisher('/img_w_path3','sensor_msgs/Image');
[pub_turnpoint, msg_turnpoint] = rospublisher('/turnpoint','std_msgs/Float32MultiArray');
[pub_parking_point, msg_parking_point] = rospublisher('/parking_point','std_msgs/Float32MultiArray');
[pub_coord_ang, msg_coord_ang] = rospublisher('/coord_ang','std_msgs/Float32MultiArray');

turn_point = [0, 0; 0, 0; 0, 0];
msg_turnpoint = rosmessage(pub_turnpoint);
msg_turnpoint.Data = turn_point;
msg_turnpoint.Layout.Dim = [ros.msggen.std_msgs.MultiArrayDimension, ros.msggen.std_msgs.MultiArrayDimension];
msg_turnpoint.Layout.Dim(1).Size = 2;
msg_turnpoint.Layout.Dim(2).Size = 3;

iter = 0;
%% subscriber
sub_droid_cam = rossubscriber('/camera/image_raw','DataFormat','struct'); % droid cam node
img_old = zeros(443, 465, 3, "uint8");


%% set the goal position
    th = 100;
    
% [x, y, theta, head] head: 00(10: front, 20: back, 1~5: parkinglot location)
% forward
%    goalPose = [0.2*th 2.4*th pi 11]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 1f
%    goalPose = [0.4*th 1.2*th pi 12];
   goalPose = [0.4*th 0.1*th pi 13]; %%forward: y value:350 // backward:
%    goalPose = [4.4*th 1.2*th 0 14]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 4f
%     goalPose = [4.4*th 0.1*th 0 15]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 5f

% backward
%     goalPose = [0.4*th 2.3*th 0 21]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 1f
 %   goalPose = [0.4*th 1.4*th 0 22]; %%forward: y value:350 // backward:
%     210 // 290 // 370 yvalue:410 2f
%      goalPose = [0.3*th 0.1*th 0 23]; %%forward: y value:350 // backward:
%     210 // 290 // 370 yvalue:410 3f
%     goalPose = [4.4*th 1.2*th pi 24]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 4f
%     goalPose = [4.4*th 0.1*th pi 25]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410 5f

%%
while 1
%     img_droid_ori = imread('./images/parkinglot.png');
    msg_droid_cam = receive(sub_droid_cam);
    img_droid_ori = rosReadImage(msg_droid_cam);
    figure(1)
%     imwrite(img_droid_ori,'parkinglot.png')
    imshow(img_droid_ori)
    if iter == 0
        [img_w_path1, img_w_path2, img_w_path3, turn_point] = hybridAstar_imgs(img_droid_ori, img_old, goalPose);
    end
%     img_w_path = hybridAstar(img_droid_ori, img_old);
    img_old = img_w_path1;
    figure(2)
    imshow(img_w_path1)
    figure(3)
    imshow(img_w_path2)
    figure(4)
    imshow(img_w_path3)

    msg_img_w_path1 = rosmessage(pub_img_w_path1);
    msg_img_w_path2 = rosmessage(pub_img_w_path2);
    msg_img_w_path3 = rosmessage(pub_img_w_path3);

    msg_img_w_path1.Encoding = 'rgb8';
    msg_img_w_path2.Encoding = 'rgb8';
    msg_img_w_path3.Encoding = 'rgb8';

    writeImage(msg_img_w_path1,img_w_path1);
    writeImage(msg_img_w_path2,img_w_path2);
    writeImage(msg_img_w_path3,img_w_path3);

    msg_turnpoint.Data = turn_point;
    msg_parking_point.Data = goalPose(1:2);
    send(pub_turnpoint, msg_turnpoint)
    send(pub_parking_point, msg_parking_point)
    send(pub_img_w_path1, msg_img_w_path1)
    send(pub_img_w_path2, msg_img_w_path2)
    send(pub_img_w_path3, msg_img_w_path3)
    
    iter =+ 1;
end
