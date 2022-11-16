clear
close
clc

rosshutdown;
rosinit('http://192.168.0.85:11311/');
% rosinit
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


% sub_x = rossubscriber('/coord_x','DataFormat','struct');
% coord_x = receive(sub_x);
% sub_y = rossubscriber('/coord_y','DataFormat','struct');
% coord_y = receive(sub_y);
% sub_ang = rossubscriber('/coord_ang','DataFormat','struct');
% 
% 
% for i = 1:size(coord_x.Data)
%     plot(coord_x.Data(i),coord_y.Data(i),'.')
%     hold on
% end
%% subscriber
sub_droid_cam = rossubscriber('/camera/image_raw','DataFormat','struct'); % droid cam node
sub_accX = rossubscriber('/arduino_imu/accX','DataFormat','struct');
sub_accY = rossubscriber('/arduino_imu/accY','DataFormat','struct');
% sub_aglX = rossubscriber('/arduino_imu/aglX','DataFormat','struct');
% sub_aglY = rossubscriber('/arduino_imu/aglY','DataFormat','struct');
sub_aglZ = rossubscriber('/arduino_imu/aglZ','DataFormat','struct');
% 
%
% 
% imshow(img_droid_ori);






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
    % xlim([-10 10]);
    % ylim([-10 10]);
dt = 0.01;
X = zeros(400,1);
Y = zeros(400,1);
x_t = zeros(400,1);
y_t = zeros(400,1);
i = 1;
while 1
%     msg_droid_cam = receive(sub_droid_cam);
%     img_droid_ori = rosReadImage(msg_droid_cam);
%     figure(1)
% %     imshow(img_droid_ori)
%     img_parkinglot = img_cal(img_droid_ori);
%     imshow(img_parkinglot)
%     msg_img_w_path = rosmessage(pub_img_w_path);
%     msg_img_w_path.Encoding = 'rgb8';
%     writeImage(msg_img_w_path,img_parkinglot);
%     send(pub_img_w_path, msg_img_w_path)

    az = receive(sub_aglZ);

    z_rad = az.Data*pi/180;
    [coord_glo, coord_t] = GeneralLineartransfrom(X(i),Y(i),0,vel,dt,z_rad);
    X(i+1) = X(i)+coord_t(1);
    Y(i+1) = Y(i)+coord_t(2);
    ang = atan2(coord_glo(2), coord_glo(1));
    x_t(i+1) = x_t(i) + vel*cos(ang)*dt;
    y_t(i+1) = y_t(i) + vel*sin(ang)*dt;
    figure(1)
    plot(X(i),Y(i),'.','Color',[1, i/350, 0])
    hold on
    figure(2)
    plot(i, z_rad,'.','Color',[i/350,0,1])
    hold on
    figure(3)
    plot(x_t(i),y_t(i),'.','color',[0,0.5,i/350])
    hold on
    
    i = i + 1;
    pause(0.1)
end
