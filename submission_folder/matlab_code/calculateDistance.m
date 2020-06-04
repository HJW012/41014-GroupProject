%% Distance between poses
% Simple function used to calculate the distance between two 3D points
function dist = calculateDistance(x1, y1, z1, x2, y2, z2)
    dist = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
end