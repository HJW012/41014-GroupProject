% Initialising the environment
%rosshutdown;
rosinit;
count = 0;
arTagSub = rossubscriber('ar_pose_marker', 'BufferSize', 25);
followerOdomSub = rossubscriber('tb3_1/odom', 'BufferSize', 25);

previousARGlobalPosition = [0 0 0];
currentARGlobalPosition = [0 0 0];

counter = 0;

% Camera x offset from turtlebot TF
camOffset = 0.0789;

% AR tags TF offset
arXOffset = 0.09; %52 mm
arYOffset = 0.050; %50 mm



% Define global offset from leader




% Search for AR Tag
localPoseData = receive(arTagSub,5);
odomData = receive(followerOdomSub, 5);

while size(localPoseData.Markers,1) < 1
    % Spin around in circles searching for AR tag in vision
    disp('Searching for first Tag');
    localPoseData = receive(arTagSub,5);
    odomData = receive(followerOdomSub, 5);
    pause(0.1);
    
end

disp('Found first tag');
% Stop spinning


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

disp(['Direction: ', num2str(direction)]);
disp(['Target Orientation: ', num2str(desiredOrientation)]);
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
        
        disp(['Direction: ', num2str(direction)]);
        disp(['Target Orientation: ', num2str(desiredOrientation)]);
        
        if direction == 2
            % Leader Stopped, stop driving

        else
            % Drive towards leader using currentARGlobalPosition, direction (1 =
            % forwards, 0 = reverse) & target orientation

        end
        
        % Rhys' Waypoint
    
    
    
        % Calculate parameters to drive to leader


        % Send velocity command to ROS

        
    else
        % Lost the AR tag, spin around to search for it again
        
        % Read latest ROS messages (odom and ar tag)
        localPoseData = receive(arTagSub,5);
        odomData = receive(followerOdomSub, 5);
       
    end
    
    % Definitely need to reduce this number, just for testing purposes
    pause(5);
end


rosshutdown;
