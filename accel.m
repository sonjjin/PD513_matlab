clear

load accel.mat
dt = 0.1;
X = 0;
Y = 0;
figure(1)
pause(0.1)
y_acc = -y_acc;
x_vel = x_acc*dt;
y_vel = y_acc*dt;
vel = 50;
ylim([-1 1])
for i = 2:3000
    theta = atan2(y_acc(i),x_acc(i));
%     acc_x_glo = x_acc(i)*cos(theta)-y_acc(i)*sin(theta);
%     acc_y_glo = x_acc(i)*sin(theta)+y_acc(i)*cos(theta); 
%     X = X + x_vel(i-1) + 1/2*acc_x_glo*dt*dt;
%     Y = Y + y_vel(i-1) + 1/2*acc_y_glo*dt*dt;
    X = X + vel*cos(theta)*dt;
    Y = Y + vel*sin(theta)*dt;
%     plot(i,theta,'.','Color','red')
    plot(X,Y,'.','Color','red')
    hold on
    pause(0.01)
end