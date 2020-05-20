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
robot = rospublisher('/cmd_vel');

goToGlobalPose(odom, robot, 0, 0, 0);
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
    pose = odomdata.Pose.Pose;
end
%% Convert quaternian to rpy
function orient = quatToRPY(pose)
    quat = [pose.Orientation.W pose.Orientation.X pose.Orientation.Y pose.Orientation.Z];
    orient = quat2eul(quat);
end
%% Go to local pose
function goToLocalPose(sub, r, x, y, z)

end
%% Go to global position
function goToGlobalPose(sub, r, x, y, z)
    while (true)
        
        
        currentPose = receivePose(sub);
        x1 = currentPose.Position.X;
        y1 = currentPose.Position.Y;
        z1 = currentPose.Position.Z;
        x2 = x;
        y2 = y;
        z2 = z;
        distanceToTarget = distance(x1, x2, y1, y2, z1, z2);
        
        linearGap = 0.1;
        angleGap = 0.001;
        angularGain = 1;
        linearGain = 1;
        angVel = 0;
        linVel = 0;
        
        if (distanceToTarget >= linearGap)
            theta = angle(x1, y1, x2, y2);
            delta = quatToRPY(currentPose); % Currently in range of -pi to pi with zero being right x axis
            
            angleToGo = abs(delta(1) - theta);
            
            % Case 1 - target above turtle
            if (y1 < y2)
                sigma = -(pi - theta);
                if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                    if (abs(angleToGo)> abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angVel = -angularGain * abs(angleToGo);  
                    end
                end
                
                if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angVel = angularGain * abs(angleToGo);
                    end
                end
            end
            
            % Case 2 - target below turtle
            if (y2 < y1)
                sigma = pi + theta;
                
                if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angVel = -angularGain * abs(angleToGo);
                    end
                end
                
                if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angVel = angularGain * abs(angleToGo);
                    end
                end
            end
            if (abs(angleToGo) <= 3 * angleGap)
                linVel = linearGain * distanceToTarget;
                angVel = 0;
            end
            
            sendVel(r, linVel, angVel);
        else
            disp("Target Reached");
            return;
        end
        
    end
end
%% Distance between poses
function dist = distance(x1, y1, z1, x2, y2, z2)
    dist = sqrt( (x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2 );
end
%% Angle between poses
function ang = angle(x1, y1, x2, y2)
    deltaX = x2-x1;
    deltaY = y2-y1;
    ang = atan2(deltaY, deltaX);
end