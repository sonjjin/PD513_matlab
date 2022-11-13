clear
close
clc

rosshutdown;
rosinit('http://192.168.0.85:11311/');
% rosinit

%%% publisher
[pub_img_w_path, msg_img_w_path] = rospublisher('/img_w_path','sensor_msgs/Image');
[pub_coord_x, msg_coord_x] = rospublisher('/coord_x','std_msgs/Float32MultiArray');
[pub_coord_y, msg_coord_y] = rospublisher('/coord_y','std_msgs/Float32MultiArray');
[pub_coord_ang, msg_coord_ang] = rospublisher('/coord_ang','std_msgs/Float32MultiArray');
msg_coord_x.Data = refpath.States(:,1);
msg_coord_y.Data = refpath.States(:,2);
msg_coord_ang.Data = refpath.States(:,3);
send(pub_coord_x, msg_coord_x);

%%% subscriber
sub_droid_cam = rossubscriber('/camera/image_raw','DataFormat','struct'); % droid cam node
% sub_accX = rossubscriber('/arduino_imu/accX','DataFormat','struct');
% sub_accY = rossubscriber('/arduino_imu/accY','DataFormat','struct');
% sub_aglX = rossubscriber('/arduino_imu/aglX','DataFormat','struct');
% sub_aglY = rossubscriber('/arduino_imu/aglY','DataFormat','struct');
% sub_aglZ = rossubscriber('/arduino_imu/aglZ','DataFormat','struct');

%%
while 1
%     img_droid_ori = imread('./images/parkinglot.png');
    msg_droid_cam = receive(sub_droid_cam);
    img_droid_ori = rosReadImage(msg_droid_cam);
    figure(1)
%     imwrite(img_droid_ori,'parkinglot.png')
    imshow(img_droid_ori)
    img_w_path = hybridAstar(img_droid_ori);
    imshow(img_w_path)
    msg_img_w_path = rosmessage(pub_img_w_path);
    msg_img_w_path.Encoding = 'rgb8';
    writeImage(msg_img_w_path,img_w_path);
    send(pub_img_w_path, msg_img_w_path)
%     x = receive(sub_accX);
%     y = receive(sub_accY);
%     ax = receive(sub_aglX);
%     ay = receive(sub_aglY);
%     az = receive(sub_aglZ);
%     z_agl(i) = az.Data;
%     z = z_agl(i)-z_agl(i-1);
%     x_acc(i) = x.Data;
%     y_acc(i) = -y.Data;
%     x_vel(i) = x_acc(i)*dt;
%     y_vel(i) = y_acc(i)*dt;
%     x_pose(i) = x_pose(i-1)+x_vel(i-1)*dt+1/2*x_acc(i)*dt*dt;
%     y_pose(i) = y_pose(i-1)+y_vel(i-1)*dt+1/2*y_acc(i)*dt*dt;
%     plot(x_pose(i), y_pose(i), '.', 'color', 'r')
%     plot(i, ax.Data,'.','Color','red')
%     hold on
%     plot(i, ay.Data,'.','Color','blue')
%     plot(i, z,'.','Color','green')
%     i = i+1;
%     pause(0.1)
end
