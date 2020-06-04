%% Angle between poses
% Simple function used to calculate the Angle between two points
function ang = calculateAngle(x1, y1, x2, y2)
    deltaX = x2-x1;
    deltaY = y2-y1;
    
    % Returns an angle from -pi to pi
    ang = atan2(deltaY, deltaX);
end