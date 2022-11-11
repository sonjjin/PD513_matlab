clear
close
clc

rosshutdown;
% rosinit('http://192.168.0.85:11311/');
rosinit
%%% initialize hybrid A star
load map2.mat;
th = 100;
map = binaryOccupancyMap(Realmap,1);
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
sv = validatorOccupancyMap(ss);
sv.Map = map;
planner = plannerHybridAStar(sv,'MinTurningRadius',1.6*th,'MotionPrimitiveLength',0.7*th);
startPose = [3*th 3.6*th pi];
goalPose = [0.5*th 0.6*th 0]; % 0: backward, pi: forward
refpath = plan(planner,startPose,goalPose);
%%% publisher
[pub_img_w_path, msg_img_w_path] = rospublisher('/img_w_path','sensor_msgs/Image');
[pub_coord_x, msg_coord_x] = rospublisher('/coord_x','std_msgs/Float32MultiArray');
[pub_coord_y, msg_coord_y] = rospublisher('/coord_y','std_msgs/Float32MultiArray');
[pub_coord_ang, msg_coord_ang] = rospublisher('/coord_ang','std_msgs/Float32MultiArray');
msg_coord_x.Data = refpath.States(:,1);
msg_coord_y.Data = refpath.States(:,2);
msg_coord_ang.Data = refpath.States(:,3);
send(pub_coord_x, msg_coord_x);
% send(pub_coord_y, msg_coord_y);
% send(pub_coord_ang, msg_coord_ang);
AAAA

sub_x = rossubscriber('/coord_x','DataFormat','struct');
coord_x = receive(sub_x);
sub_y = rossubscriber('/coord_y','DataFormat','struct');
coord_y = receive(sub_y);
sub_ang = rossubscriber('/coord_ang','DataFormat','struct');


for i = 1:size(coord_x.Data)
    plot(coord_x.Data(i),coord_y.Data(i),'.')
    hold on
end
%%% subscriber
% sub_droid_cam = rossubscriber('/camera/image_raw','DataFormat','struct'); % droid cam node
% sub_accX = rossubscriber('/arduino_imu/accX','DataFormat','struct');
% sub_axxY = rossubscriber('/arduino_imu/accY','DataFormat','struct');
% sub_aglZ = rossubscriber('/arduino_imu/aglZ','DataFormat','struct');
% 
% 
% droid_cam_msg = receive(droid_cam_sub, 3);
% img_droid_ori = rosReadImage(droid_cam_msg);
% img_parkinglot = img_cal(img_droid_cam);
% 
% imshow(img_droid_ori);

% img_msg = rosmessage(img_pub);
% 
% 
% 
% img_msg.Encoding = 'rgb8';
% writeImage(img_msg,img);
% send(img_pub, img_msg)




% sub = rossubscriber('/ctrl_motor');
% sub = rossubscriber('/image_view/parameter_descriptions','DataFormat','struct');
% sub2 = rossubscriber('/img');
% pause(1)
% 
% % msg = rosmessage(sub);
% 
% 
% % msg = rosmessage('sensor_msgs/Image');
% % msg.Encoding = 'rgb8';
% % writeImage(msg,img);
% 
% while 1
% 
% %     img_msg.Encoding = 'rgb8';
% %     writeImage(img_msg,img);
% %     send(img_pub, img_msg);
% %     pause(0.1);
% end
