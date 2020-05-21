% Initialising the environment

% Define global offset from leader




% Search for AR Tag

while size(arTagMsg.Markers,1) < 1
    % Spin around in circles searching for AR tag in vision
    arTagMsg = receive(arTagSub,5);
    followerMsg = receive(followerOdomSub, 5);
    
end



% Main Loop
while(1)
    % Read latest ROS messages (odom and ar tag)
    
    
    % Interpolate Orientation
    
    
    
    
    % Rhys' Waypoint
    
    
    
    % Calculate parameters to drive to leader
    
    
    % Send velocity command to ROS
    
end



