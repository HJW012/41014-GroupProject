% Initialising the environment
%rosshutdown;
rosinit;
count = 0;
arTagSub = rossubscriber('ar_pose_marker', 'BufferSize', 25);
followerOdomSub = rossubscriber('tb3_1/odom', 'BufferSize', 25);
robotVelocity = rospublisher('tb3_1/cmd_vel');

previousARGlobalPosition = [0 0 0];
currentARGlobalPosition = [0 0 0];

counter = 0;

% Camera x offset from turtlebot TF
camOffset = 0.0789;

% AR tags TF offset
arXOffset = 0.09; %52 mm
arYOffset = 0.050; %50 mm

% Define global offset from leader
followerOffset = 0.2; % 20 cm

% Search for AR Tag
localPoseData = receive(arTagSub,5);
odomData = receive(followerOdomSub, 5);

while size(localPoseData.Markers,1) < 1
    % Spin around in circles searching for AR tag in vision
    sendVel(robotVelocity, 0, 1.5);
        
    
    disp('Searching for first Tag');
    localPoseData = receive(arTagSub,5);
    odomData = receive(followerOdomSub, 5);
    pause(0.1);
    
end

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
    localPoseData = receive(arTagSub,5);
    odomData = receive(followerOdomSub, 5);
    
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
[desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(3));

% Calculate target position offset from leader
targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];


disp(['Direction: ', num2str(direction)]);
disp(['Target Orientation: ', num2str(desiredOrientation)]);
disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
disp(['Target Position: ', num2str(targetGlobalPosition)]);
if direction == 2
    % Leader Stopped, stop driving
    
else
    % Drive towards leader using currentARGlobalPosition, direction (1 =
    % forwards, 0 = reverse) & target orientation
    
end



% Main Loop
while(1)
    disp('In Main Loop');
    % Read latest ROS messages (odom and ar tag)
    localPoseData = receive(arTagSub,5);
    odomData = receive(followerOdomSub, 5);
    
    if size(localPoseData.Markers,1) > 0
        % Store second Pose
        previousARGlobalPosition = currentARGlobalPosition;
        previousOrientation = desiredOrientation;
        
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

        % Interpolate Orientation
        [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(3));
        
                      
        if direction == 2
            % Leader Stopped, stop driving if already following the robot
            % at the required distance
            
            
            % Maintain current heading
            desiredOrientation = previousOrientation;
            
            % Calculate target position offset from leader
            targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];
            
            [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation, targetGlobalPosition, desiredOrientation);

        else
            % Calculate target position offset from leader
            targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];
            
            % Drive towards leader using targetGlobalPosition, direction (1 =
            % forwards, 0 = reverse) & target orientation
            [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation, targetGlobalPosition, desiredOrientation);


        end
        
        disp(['Direction: ', num2str(direction)]);
        disp(['Target Orientation: ', num2str(desiredOrientation)]);
        disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
        disp(['Target Position: ', num2str(targetGlobalPosition)]);
        
        % Rhys' Waypoint
    
    
    
        % Calculate parameters to drive to leader


        % Send velocity command to ROS
        sendVel(robotVelocity, linearVel, angularVel);

        
    else
        % Lost the AR tag, spin around to search for it again
        disp('Lost AR Tag, searching again');
        sendVel(robotVelocity, 0, 0.5);
        
        
        % Read latest ROS messages (odom and ar tag)
        localPoseData = receive(arTagSub,5);
        odomData = receive(followerOdomSub, 5);
       
    end
    
    pause(0.5);
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
        
        linearGap = 0.2;
        angleGap = 0.001;
        angularGain = 1;
        linearGain = 1;
        angularVel = 0;
        linearVel = 0;
        
        if (distanceToTarget >= linearGap)
            theta = angle(x1, y1, x2, y2);
            delta = currentOrientation; % Currently in range of -pi to pi with zero being right x axis
            
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
            if (abs(angleToGo) <= 50 * angleGap)
                linearVel = linearGain * distanceToTarget;
                angularVel = 0;
            end
            
            %sendVel(r, linVel, angVel);
        else
            disp("Target Reached");
            return;
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