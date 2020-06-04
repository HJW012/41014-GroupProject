% Initialising the environment
clc;
clear;

pause(2);

% Initialising ROS environment and setting up required subscribers and
% publishers
rosinit;
arTagSub = rossubscriber('ar_pose_marker', 'BufferSize', 25);
followerOdomSub = rossubscriber('tb3_1/odom', 'BufferSize', 25);
robotVelocity = rospublisher('tb3_1/cmd_vel');

% Initialising counters and required variables
count = 0;
timeOut = 10;
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

% Initialising Lost AR Value
lostAR = 0;

% While the robot can't see the tag, rotate on the spot until the tag is
% seen in camera frame
while size(localPoseData.Markers,1) < 1
    lostAR = 1;
    % Spin around in circles searching for AR tag in vision
    sendVel(robotVelocity, 0, 1);
    
    disp('Searching for first Tag');
    
    % Receive data to check again
    localPoseData = receive(arTagSub,timeOut);
    odomData = receive(followerOdomSub, timeOut);
    
    % Slow the reading down a bit
    pause(0.1);
end

lostAR = 0;

disp('Found first tag');

% Stop spinning
sendVel(robotVelocity, 0, 0);

% Store the current pose of the follower turtlebot
tbPose = odomData.Pose.Pose.Position;
tbQuat = odomData.Pose.Pose.Orientation;
tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)

% Store the latest data from the AR Tag tracking package
currentLocalPose = localPoseData.Markers.Pose.Pose;
currentOffsetLocalPose = currentLocalPose;

% Apply offset values to the data to cater for the position of the bottom
% left tag to the middle of the tag bundle as well as the relative
% positioning of the camera to the centre of the follower turtlebot
currentOffsetLocalPose.Position.X = currentOffsetLocalPose.Position.X + arXOffset;
currentOffsetLocalPose.Position.Y = currentOffsetLocalPose.Position.Y - arYOffset;
currentOffsetLocalPose.Position.Z = currentOffsetLocalPose.Position.Z + camOffset;

% Convert the data obtained from the camera into a Global position of the
% leader Turtlebot
currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);


counter = counter + 1;

pause(1);

% Wait to obtain a second reading of the tag before trying to interpolate
% the orientation of the leader Turtlebot
while counter < 2
    disp('Waiting for second Tag');
    
    % Obtain the latest data from the required subscribers
    localPoseData = receive(arTagSub,timeOut);
    odomData = receive(followerOdomSub, timeOut);
    
    % Check that the data obtained actually contains usable data
    if size(localPoseData.Markers,1) > 0
        disp('Found second Tag');
        
        % Store second Pose
        previousARGlobalPosition = currentARGlobalPosition;
        
        % Store the current pose of the follower turtlebot
        tbPose = odomData.Pose.Pose.Position;
        tbQuat = odomData.Pose.Pose.Orientation;
        tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)

        % Store the latest data from the AR Tag tracking package
        currentLocalPose = localPoseData.Markers.Pose.Pose;
        currentOffsetLocalPose = currentLocalPose;
        
        % Apply offset values to the data to cater for the position of the bottom
        % left tag to the middle of the tag bundle as well as the relative
        % positioning of the camera to the centre of the follower turtlebot
        currentOffsetLocalPose.Position.X = currentOffsetLocalPose.Position.X + arXOffset;
        currentOffsetLocalPose.Position.Y = currentOffsetLocalPose.Position.Y - arYOffset;
        currentOffsetLocalPose.Position.Z = currentOffsetLocalPose.Position.Z + camOffset;

        % Convert the data obtained from the camera into a Global position of the
        % leader Turtlebot
        currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);

        % Update counter
        counter = counter + 1;
    end
    
    pause(0.1);
end

% Interpolate Leader Orientation
[desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(1), pi/2);

% Calculate target position offset from leader
targetGlobalPosition = [(currentARGlobalPosition.Position.X - (followerOffset*(cos(desiredOrientation)))), (currentARGlobalPosition.Position.Y - (followerOffset*(sin(desiredOrientation)))), 0];


if direction == 2
    % Leader Stopped,
    direction = previousDirection;
    
    % Calculate the drive parameters required to drive towards target
    % location and move follower turtlebot in that direction
    [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);
    
else
    % Calculate the drive parameters required to drive towards target
    % location and move follower turtlebot in that direction
    [linearVel, angularVel] = calculateDriveParams(tbPose, tbOrientation(1), targetGlobalPosition, desiredOrientation);

end

disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
disp(['Target Position: ', num2str(targetGlobalPosition)]);

% Drive the follower Turtlebot
sendVel(robotVelocity, linearVel, angularVel);

% Main Loop
while(1)
    % Only re-calculate the leader position every 15 times, otherwise the
    % target position and interpolated orientation are too noisy
    if count == 15
        count = 0;
        
        % Read latest ROS messages (odom and ar tag)
        localPoseData = receive(arTagSub,timeOut);
        odomData = receive(followerOdomSub, timeOut);

        % Check that the AR Tag ROS message is valid
        if size(localPoseData.Markers,1) > 0
            
            % If the AR Tag was previously lost, stop the robot from
            % spinning
            if lostAR == 1
                count = 0;
                disp('Found AR Tag again');
                % Stop follower robot
                sendVel(robotVelocity, 0, 0);
                lostAR = 0;                
            end
            
            % Store previous Leader Pose data to be used for the
            % interpolation
            previousARGlobalPosition = currentARGlobalPosition;
            previousOrientation = desiredOrientation;
            previousDirection = direction;

            % Store the current pose of the follower turtlebot
            tbPose = odomData.Pose.Pose.Position;
            tbQuat = odomData.Pose.Pose.Orientation;
            tbOrientation = quat2eul([tbQuat.W tbQuat.X tbQuat.Y tbQuat.Z]); %Rot on z is (1)
            
            % Store latest data from the AR Tag tracking package
            currentLocalPose = localPoseData.Markers.Pose.Pose;
            currentOffsetLocalPose = currentLocalPose;
            
            % Apply offset values to the data to cater for the position of the bottom
            % left tag to the middle of the tag bundle as well as the relative
            % positioning of the camera to the centre of the follower turtlebot
            currentOffsetLocalPose.Position.X = currentOffsetLocalPose.Position.X + arXOffset;
            currentOffsetLocalPose.Position.Y = currentOffsetLocalPose.Position.Y - arYOffset;
            currentOffsetLocalPose.Position.Z = currentOffsetLocalPose.Position.Z + camOffset;

            % Convert the data obtained from the camera into a Global position of the
            % leader Turtlebot
            currentARGlobalPosition = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation);

            % Interpolate Orientation
            [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation(1), previousOrientation);


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

            disp(['Leader Position: ', num2str(currentARGlobalPosition.Position.X), ', ', num2str(currentARGlobalPosition.Position.Y)]);
            disp(['Target Position: ', num2str(targetGlobalPosition)]);
            
            % Send velocity command to follower Turtlebot through ROS
            sendVel(robotVelocity, linearVel, angularVel);

        else
            % Lost the AR tag, spin around to search for it again
            disp('Lost AR Tag, searching again');
            lostAR = 1;
            
            % Store the global y values of the follower and leader robot in
            % temp variables
            y1 = tbPose.Y;
            y2 = targetGlobalPosition(2);
            
            % Calculate the angle between the follower and leader positions 
            theta = calculateAngle(tbPose.X, tbPose.Y, targetGlobalPosition(1), targetGlobalPosition(2));

            delta = tbOrientation(1); % Currently in range of -pi to pi with zero being right x axis
                        
            % Case 1 - target above turtle
            if (y1 < y2)
                sigma = -(pi - theta);
                if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                    % Rotate counter-clockwise    
                    angularVel = -1;
                end
                
                if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                    % Rotate clockwise    
                    angularVel = 1;
                end
            end
            
            % Case 2 - target below turtle
            if (y2 < y1)
                sigma = pi + theta;
                
                if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                    % Rotate counter-clockwise    
                    angularVel = -1;
                end
                
                if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                    % Rotate clockwise    
                    angularVel = 1;
                end
            end
            
            % Stop the follower from driving forward/reverse and simply
            % spin on the spot to re-locate the AR Tag
            linearVel = 0;
            
            % Send velocity command to follower Turtlebot through ROS
            sendVel(robotVelocity, linearVel, angularVel);

            % Read latest ROS messages (odom and ar tag)
            localPoseData = receive(arTagSub,timeOut);
            odomData = receive(followerOdomSub,timeOut);
            
            % Set count to 15 so that this code will run every time so the
            % AR Tag is not missed
            count = 15;
        end

        pause(0.1);
        
    else
        % Continue to re-calculate the required drive parameters drive 
        % at the calculated target position
        
        % Read latest odometry data from the follower robot
        odomData = receive(followerOdomSub, timeOut);
        
        % Store the current pose of the follower turtlebot
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
