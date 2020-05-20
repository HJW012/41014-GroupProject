rosshutdown;
clear;
clc;

ipaddress = 'http://192.168.46.130:11311';
rosinit(ipaddress);

%handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
%receive(handles.odomSub, 3);
%handles.laserSub = rossubscriber('/scan', 'BufferSize', 5);
%receive(handles.laserSub, 3);

%handles.velPub = rospublisher('/cmd_vel');


odom = rossubscriber('/odom', 'BufferSize', 25);
tagLocalPose = rossubscriber('/ar_pose_marker', 'BufferSize', 25);


odomData = receive(odom, 3);
localPoseData = receive(tagLocalPose, 3);

% Camera x offset from turtlebot TF
camOffset = 0.0789;

% AR tags TF offset
arXOffset = 0.09; %52 mm
arYOffset = 0.050; %50 mm

% Current TB global TF
tbPose = odomData.Pose.Pose.Position;
tbQuat = odomData.Pose.Pose.Orientation;
tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)

% Local Pose TF
currentLocalPose = localPoseData.Markers.Pose.Pose;
currentOffsetLocalPose = currentLocalPose;
currentOffsetLocalPose.Position.X = currentOffsetLocalPose.Position.X + arXOffset;
currentOffsetLocalPose.Position.Y = currentOffsetLocalPose.Position.Y - arYOffset;
currentOffsetLocalPose.Position.Z = currentOffsetLocalPose.Position.Z + camOffset;

% Distance between TB and ARTag
distance = abs(sqrt( (currentOffsetLocalPose.Position.Z)^2 + (currentOffsetLocalPose.Position.X)^2 ));



% Convert to ARTag to global pose
arGlobalPose.Position.Z = -currentOffsetLocalPose.Position.Y; %Not totally correct - dont take height of camera into account yet
originX = tbPose.X;
originY = tbPose.Y;
zRot = tbOrientation(1);

if (zRot > pi/2)
   if (currentOffsetLocalPose.Position.X < 0) % 1
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = zRot;
       gamma = pi - (alpha + theta);
       
       arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 1");
   elseif (currentOffsetLocalPose.Position.X > 0) % 2
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = zRot;
       gamma = pi - (theta - alpha);
       
       arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 2");
   end
end

if (zRot > 0 && zRot < pi/2)
    if (currentOffsetLocalPose.Position.X < 0) % 3
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = zRot;
       gamma = (alpha + theta);
       
       arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 3");
   elseif (currentOffsetLocalPose.Position.X > 0) % 4
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = zRot;
       gamma = (theta - alpha);
       
       arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 4");
   end
end

if (zRot < 0 && zRot > -pi/2)
    if (currentOffsetLocalPose.Position.X < 0) % 5
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = abs(zRot);
       gamma = (theta - alpha);
       
       arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 5");
   elseif (currentOffsetLocalPose.Position.X > 0) % 6
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = abs(zRot);
       gamma = (theta + alpha);
       
       arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 6");
   end
end

if (zRot < -pi/2)
   if (currentOffsetLocalPose.Position.X < 0) % 7
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = abs(zRot);
       gamma = pi - (theta - alpha);
       
       arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 7");
   elseif (currentOffsetLocalPose.Position.X > 0) % 8
       alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
       theta = abs(zRot);
       gamma = pi - (theta + alpha);
       
       arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
       arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
       disp("Case 8");
   end 
end


% Get distance
% Use trig to get cx and y offsets, making sure to change reference frame
% for tag to robot
%% 
%rosshutdown;
%clear;
%clc;

%ipaddress = 'http://192.168.46.130:11311';
%rosinit(ipaddress);

%folderpath = '~/catkin_ws/src/ar_track_alvar';
%rosgenmsg(folderpath);

addpath('/home/hwittke/catkin_ws/src/ar_track_alvar/matlab_gen/msggen')
savepath
%% Receive Pose
function pose = receivePose(sub)
    odomdata = receive(sub, 3);
    pose = odomdata.Pose.Pose;
end
%% Convert local pose to global
