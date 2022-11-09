clear
close
clc

rosshutdown;
rosinit('http://192.249.30.150:11311/');
node = ros.Node('/test');
% clear('pub','sub','node')
clear('master')
pub = rospublisher('/ctrl_cmd2','std_msgs/Int32');
sub = rossubscriber('/ctrl_cmd2','std_msgs/Int32','DataFormat','struct');
while 1
    i = 0
    msg = rosmessage(pub);
    msg.Data = i;
    send(pub,msg)
    i = i+1;
    x = receive(sub, 5);
    display(x)
    pause(1)
end
sub.LatestMessage