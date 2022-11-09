clear
close
clc
img = imread('test.jpeg');
rosshutdown;
rosinit('http://192.168.0.85:11311/');
%% publisher
[img_pub, img_msg] = rospublisher('/img_w_path','sensor_msgs/Image');

%% subscriber
droid_cam_sub = rossubscriber('/camera/image_raw','DataFormat','struct'); % droid cam node


img_msg.Encoding = 'rgb8';
writeImage(img_msg,img);
send(img_pub, img_msg)




% sub = rossubscriber('/ctrl_motor');
% sub = rossubscriber('/image_view/parameter_descriptions','DataFormat','struct');
% sub2 = rossubscriber('/img');
pause(1)

% msg = rosmessage(sub);


% msg = rosmessage('sensor_msgs/Image');
% msg.Encoding = 'rgb8';
% writeImage(msg,img);

while 1
    msg = receive(sub, 3);
    image = rosReadImage(msg);
    imshow(image);
    img_msg = rosmessage(img_pub);
%     img_msg.Encoding = 'rgb8';
%     writeImage(img_msg,img);
%     send(img_pub, img_msg);
%     pause(0.1);
end
