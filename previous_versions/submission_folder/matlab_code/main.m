% Initialising the environment
clc;
clear;

pause(2);


rosinit;
count = 0;
timeOut = 10;
arTagSub = rossubscriber('ar_pose_marker', 'BufferSize', 25);
followerOdomSub = rossubscriber('tb3_1/odom', 'BufferSize', 25);
robotVelocity = rospublisher('tb3_1/cmd_vel');

previousARGlobalPosition = [0 0 0];
currentARGlobalPosition = [0 0 0];
previousDirection = 1;

counter = 0;

% Camera x offset from turtlebot TF
camOffset = 0.0789;

% AR tags TF offset
arXOffset = 0.09; %52 mm
arYOffset = 0.050; %50 mm

% Define global offset from leader
followerOffset = 0.5; % 50 cm

% Search for AR Tag
localPoseData = receive(arTagSub,timeOut);
odomData = receive(followerOdomSub, timeOut);

lostAR = 0;

while size(localPoseData.Markers,1) < 1
    lostAR = 1;
    % Spin around in circles searching for AR tag in vision
    sendVel(robotVelocity, 0, 1);
    
    disp('Searching for first Tag');
    localPoseData = receive(arTagSub,timeOut);
    odomData = receive(followerOdomSub, timeOut);
    pause(0.1);
    
end

lostAR = 0;

disp('Found first tag');
% Stop spinning
sendVel(robotVelocity, 0, 0);

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

currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);

counter = counter + 1;

pause(1);

while counter < 2
    disp('Waiting for second Tag');
    localPoseData = receive(arTagSub,timeOut);
    odomData = receive(followerOdomSub, timeOut);
    
    if size(localPoseData.Markers,1) > 0
        % Store second Pose
        previousARGlobalPosition = currentARGlobalPosition;
        
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

        currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);

        % Update counter
        counter = counter + 1;
    end
    
    pause(0.1);
end

% Interpolate Leader Orientation
[desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(1), pi/2, previousDirection);

% Calculate target position offset from leader
targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];


disp(['Direction: ', num2str(direction)]);
disp(['Target Orientation: ', num2str(desiredOrientation)]);
disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
disp(['Target Position: ', num2str(targetGlobalPosition)]);
if direction == 2
    % Leader Stopped,
    direction = previousDirection;
    
    [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);
    
else
    % Drive towards leader using currentARGlobalPosition, direction (1 =
    % forwards, 0 = reverse) & target orientation
    [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);

end

sendVel(robotVelocity, linearVel, angularVel);

count = 0;
% Main Loop
while(1)
    if count == 15
        count = 0;
        
        % Read latest ROS messages (odom and ar tag)
        localPoseData = receive(arTagSub,timeOut);
        odomData = receive(followerOdomSub, timeOut);

        if size(localPoseData.Markers,1) > 0
            if lostAR == 1
                count = 0;
                disp('Found AR Tag again');
                linearVel = 0;
                angularVel = 0;
                sendVel(robotVelocity, linearVel, angularVel);
                lostAR = 0;                
            end
            
            % Store previous Pose
            previousARGlobalPosition = currentARGlobalPosition;
            previousOrientation = desiredOrientation;
            previousDirection = direction;

            % Current TB global TF
            tbPose = odomData.Pose.Pose.Position;
            tbQuat = odomData.Pose.Pose.Orientation;
            tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)
            
            currentLocalPose = localPoseData.Markers.Pose.Pose;
            currentOffsetLocalPose = currentLocalPose;
            currentOffsetLocalPose.Position.X = currentOffsetLocalPose.Position.X + arXOffset;
            currentOffsetLocalPose.Position.Y = currentOffsetLocalPose.Position.Y - arYOffset;
            currentOffsetLocalPose.Position.Z = currentOffsetLocalPose.Position.Z + camOffset;

            currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);

            % Interpolate Orientation
            [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(1), previousOrientation, previousDirection);


            if direction == 2
                % Leader Stopped
                direction = previousDirection;
                % Calculate target position offset from leader
                targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];

                % Calculate parameters to drive to leader
                [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);

            else
                % Calculate target position offset from leader
                targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];

                % Calculate parameters to drive to leader
                [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);

            end

            disp(['Direction: ', num2str(direction)]);
            disp(['Target Orientation: ', num2str(desiredOrientation)]);
            disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
            disp(['Target Position: ', num2str(targetGlobalPosition)]);

            % Send velocity command to ROS
            sendVel(robotVelocity, linearVel, angularVel);

        else
            % Lost the AR tag, spin around to search for it again
            disp('Lost AR Tag, searching again');
            lostAR = 1;
            
            y1 = tbPose.Y;
            y2 = targetGlobalPosition(2);
            
            theta = angle(tbPose.X, tbPose.Y, targetGlobalPosition(1), targetGlobalPosition(2));

            delta = tbOrientation(1); % Currently in range of -pi to pi with zero being right x axis
                        
            % Case 1 - target above turtle
            if (y1 < y2)
                sigma = -(pi - theta);
                if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                        angularVel = -1;
                end
                
                if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                        angularVel = 1;
                end
            end
            
            % Case 2 - target below turtle
            if (y2 < y1)
                sigma = pi + theta;
                
                if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                        angularVel = -1;
                end
                
                if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                        angularVel = 1;
                end
            end
            
            linearVel = 0;
            sendVel(robotVelocity, linearVel, angularVel);

            % Read latest ROS messages (odom and ar tag)
            localPoseData = receive(arTagSub,timeOut);
            odomData = receive(followerOdomSub,timeOut);
            
            count = 15;

        end

        pause(0.1);
        
    else
        odomData = receive(followerOdomSub, timeOut);
        
        tbPose = odomData.Pose.Pose.Position;
        tbQuat = odomData.Pose.Pose.Orientation;
        tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)
        
        % Calculate parameters to drive to leader
        [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);

        % Send velocity command to ROS
        sendVel(robotVelocity, linearVel, angularVel);
        
        count = count + 1;
        pause(0.1);
        
    end
    
end


rosshutdown;


%% Send Vel Instruction
function sendVel(r, linearVel, angularVel)
    velmsg = rosmessage(r);
    velmsg.Linear.X = linearVel;
    velmsg.Angular.Z = angularVel;
    send(r, velmsg);
end


function [linearVel, angularVel] = calculateDriveParams(currentPose, currentOrientation, targetGlobalPose, targetOrientation) 
        
        x1 = currentPose.X;
        y1 = currentPose.Y;
        z1 = currentPose.Z;
        x2 = targetGlobalPose(1);
        y2 = targetGlobalPose(2);
        z2 = targetGlobalPose(3);
        distanceToTarget = distance(x1, y1, z1, x2, y2, z2);
        
        disp(['Follower Position: ', num2str(x1) ', ', num2str(y1)]);
        disp(['Target Position: ', num2str(x2), ', ', num2str(y2)]);
        
        linearGap = 0.1;
        angleGap = 0.001;
        angularGain = 0.75;
        linearGain = 0.5;
        angularVel = 0;
        linearVel = 0;
        
        if (distanceToTarget >= linearGap)
            theta = angle(x1, y1, x2, y2);
            delta = currentOrientation; % Currently in range of -pi to pi with zero being right x axis
            
            diffAngle = theta - delta;
            
            if ((-1*(deg2rad(110)) < diffAngle) && (diffAngle < deg2rad(110)))
                % Target position in front of follower
                disp('Follower driving forward');
                direction = 1;
                
            else
                % Target position behind follower
                disp('Follower driving reverse');
                direction = -1;
                angularGain = 1.5;
                linearGain = 2.5;

                % Maintain heading in line with the target point but drive backwards
                if theta <= 0
                    theta = theta + pi;
                else
                    theta = theta - pi;
                end
            end
                      
            angleToGo = abs(delta(1) - theta);
            
            % Case 1 - target above turtle
            if (y1 < y2)
                sigma = -(pi - theta);
                if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                    if (abs(angleToGo)> abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angularVel = -angularGain * abs(angleToGo);
                    end
                end
                
                if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angularVel = angularGain * abs(angleToGo);
                    end
                end
            end
            
            % Case 2 - target below turtle
            if (y2 < y1)
                sigma = pi + theta;
                
                if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angularVel = -angularGain * abs(angleToGo);
                    end
                end
                
                if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                        angularVel = angularGain * abs(angleToGo);
                    end
                end
            end
            
            if direction < 0
                if (abs(angleToGo) <= 700 * angleGap)
                    linearVel = direction * linearGain * (distanceToTarget + 0.3);
                end
            else
                if (abs(angleToGo) <= 500 * angleGap)
                    linearVel = direction * linearGain * (distanceToTarget + 0.3);
                end
                
            end

        else
            disp("Target Reached");
            
            theta = targetOrientation;
            delta = currentOrientation;
            
            angleToGo = theta - delta;
            
            if (abs(angleToGo) <= 200 * angleGap)
                disp('Stopped Moving');
                linearVel = 0;
                angularVel = 0;
                
            else
                linearVel = 0;
                
                % Case 1 - target above turtle
                if (y1 < y2)
                    sigma = -(pi - theta);
                    if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                        if (abs(angleToGo)> abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                            angularVel = -angularGain * abs(angleToGo);
                        end
                    end

                    if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                        if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                            angularVel = angularGain * abs(angleToGo);
                        end
                    end
                end

                % Case 2 - target below turtle
                if (y2 < y1)
                    sigma = pi + theta;

                    if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                        if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                            angularVel = -angularGain * abs(angleToGo);
                        end
                    end

                    if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                        if (abs(angleToGo) > abs(angleGap)) % Dont think this should work for every scenario due to -pi to pi range
                            angularVel = angularGain * abs(angleToGo);
                        end
                    end
                end
                
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


