rosinit;
count = 0;
arTagSub = rossubscriber('ar_pose_marker');
followerOdomSub = rossubscriber('odom');

previousPosition = [0 0 0];
currentPosition = [0 0 0];


arTagMsg = receive(arTagSub,5);
followerMsg = receive(followerOdomSub, 5);

while size(arTagMsg.Markers,1) < 1
    % Spin around in circles searching for AR tag in vision
    arTagMsg = receive(arTagSub,5);
    followerMsg = receive(followerOdomSub, 5);
    
end

disp('Storing first position');

% Store First Pose
currentPosition = [arTagMsg.Markers.Pose.Pose.Position.X, arTagMsg.Markers.Pose.Pose.Position.Y, arTagMsg.Markers.Pose.Pose.Position.Z];
currentRobotPosition = [followerMsg.Pose.Pose.Position.X, followerMsg.Pose.Pose.Position.Y, followerMsg.Pose.Pose.Position.Z];
currentRobotOrientation = quat2eul([followerMsg.Pose.Pose.Orientation.X, followerMsg.Pose.Pose.Orientation.Y, followerMsg.Pose.Pose.Orientation.Z, followerMsg.Pose.Pose.Orientation.W]);
currentGlobalPosition = convertToGlobal(currentPosition, currentRobotPosition, currentRobotOrientation);

pause(10);

% Stop spinning

% Update count
count = count + 1;

while count < 2
    arTagMsg = receive(arTagSub,5);
    followerMsg = receive(followerOdomSub, 5);
    
    if size(arTagMsg.Markers,1) > 0
        % Store second Pose
        previousPosition = currentPosition;
        previousGlobalPosition = currentGlobalPosition;
        
        currentPosition = [arTagMsg.Markers.Pose.Pose.Position.X, arTagMsg.Markers.Pose.Pose.Position.Y, arTagMsg.Markers.Pose.Pose.Position.Z];
        currentRobotPosition = [followerMsg.Pose.Pose.Position.X, followerMsg.Pose.Pose.Position.Y, followerMsg.Pose.Pose.Position.Z];
        currentRobotOrientation = quat2eul([followerMsg.Pose.Pose.Orientation.X, followerMsg.Pose.Pose.Orientation.Y, followerMsg.Pose.Pose.Orientation.Z, followerMsg.Pose.Pose.Orientation.W]);
        currentGlobalPosition = convertToGlobal(currentPosition, currentRobotPosition, currentRobotOrientation);
        
        % Update count
        count = count + 1;
        
    end
    
end

diffGlobalPosition = currentGlobalPosition - previousGlobalPosition;

if (((abs(diffGlobalPosition(1)) > 0.01)) || (abs(diffGlobalPosition(2)) > 0.01))
    leaderOrientation = atan2(diffGlobalPosition(2), diffGlobalPosition(1));

    diffOrientation = leaderOrientation - currentRobotOrientation(3);
    
    if ((-3*pi/4 < diffOrientation) && (diffOrientation < 3*pi/4))
        % Leader driving forward
        direction = 1;
        
    else
        direction = 0;
        
        if leaderOrientation <= 0
            desiredOrientation = leaderOrientation + pi;
            
        else
            desiredOrientation = leaderOrientation - pi;
            
        end
        
    end
    
else
    % Leader robot has stopped and maintain previous leader orientation
    desiredOrientation = leaderOrientation;
end


rosshutdown;


function globalPosition = convertToGlobal(arTagPosition, robotPosition, robotOrientation)
       
    % AR Tag z-direction is parallel with Turtlebot x-direction
    globalPositionX = robotPosition(1) + ((((arTagPosition(3) + 0.05))*(cos(robotOrientation(3)))) - ((-1*arTagPosition(1))*(sin(robotOrientation(3)))))
    globalPositionY = robotPosition(2) - ((-1*((arTagPosition(3) + 0.05))*(sin(robotOrientation(3)))) - ((-1*arTagPosition(1))*(cos(robotOrientation(3)))))
    globalPositionZ = 0;
    
    globalPosition = [globalPositionX, globalPositionY, globalPositionZ]
    
end



