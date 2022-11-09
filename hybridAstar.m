clear
% close all
clf
clc
load map2.mat
figure(4);
% mesh(Realmap)
img = calibration();
th = 100;
th0 = 1;
map = binaryOccupancyMap(Realmap,th0);
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits;[-pi pi]];
sv = validatorOccupancyMap(ss);
sv.Map = map;
% load parkingLotCostVal.mat % costVal
% map = binaryOccupancyMap(costVal);
show(map)
hold on

planner = plannerHybridAStar(sv,'MinTurningRadius',160,'MotionPrimitiveLength',70);
% planner = plannerRRTStar(validator);

startPose = [3*th 3.6*th pi];
plot(startPose(1),startPose(2),'.')
goalPose = [0.5*th 0.6*th 0]; %%forward: y value:350 // backward: 210 // 290 // 370 yvalue:410
refpath = plan(planner,startPose,goalPose);     
% refpath = plan(planner,startPose,goalPose);
figure(1)
show(planner)
figure(2);
flip_img2 = flipud(img);
imshow(flip_img2)
hold on
plot((refpath.States(:,1)),(refpath.States(:,2)),'.','Color','red')
