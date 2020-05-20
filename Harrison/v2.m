%% Version 2 of turtlebot driving to location
rosshutdown;
clear;
clc;

ipaddress = 'http://192.168.46.130:11311'; %IP address of turtlebot sim
rosinit(ipaddress);

odom = rossubscriber('/tb3_1/odom', 'BufferSize', 25);
robot = rospublisher('/tb3_1/cmd_vel', 'queue_size', 10);

targetLocation = transl(0, 0, 0);

%goToGlobalPose(odom, robot, 0, 0, 0);

%% Send Vel Instruction
function sendVel(r, linearVel, angularVel)
    velmsg = rosmessage(r);
    velmsg.Linear.X = linearVel;
    velmsg.Angular.Z = angularVel;
    send(r, velmsg);
end
%% Receive Pose
function pose = receivePose(sub)
    odomdata = receive(sub, 3);
    pose = odomdata.Pose.Pose
end
%% 2D Distance
function dist = distance2D(pose1, pose2)
    x1 = pose1(1, 4);
    y1 = pose1(2, 4);
    x2 = pose2(1, 4);
    y2 = pose2(2, 4);
    
    dist = abs(sqrt( (x1-x2)^2 + (y1-y2)^2 ));
end
%% Go To Global Pose
%function 