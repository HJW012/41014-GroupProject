%% Send Vel Instruction
% Simple function used to update the drive parameters of the follower
% Turtlebot in the ROS simulation
function sendVel(r, linearVel, angularVel)
    % Establish the ros message
    velmsg = rosmessage(r);
    
    % Store the required values in the message
    velmsg.Linear.X = linearVel;
    velmsg.Angular.Z = angularVel;
    
    % Send the message to the ROS environment
    send(r, velmsg);
end