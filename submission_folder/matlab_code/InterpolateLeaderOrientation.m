%% Interpolate Leader Orientation
% Function used to interpolate the global orientation of the leader using
% the two latest global positions of the leader robot

function [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation, previousOrientation)
    
    % Calculate the distance travelled by the leader between the last two
    % measurements
    distance = sqrt((currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X)^2 + (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y)^2 );
    
    diffGlobalPosition = [(currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X), (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y), (currentARGlobalPosition.Position.Z - previousARGlobalPosition.Position.Z)];

    % Determine if robot has moved or not
    if (distance > 0.05)
        % Calculate the orientation of the leader
        leaderOrientation = atan2(diffGlobalPosition(2), diffGlobalPosition(1));

        % Calculate difference between leader and follower orientation
        diffOrientation = leaderOrientation - tbOrientation;

        if ((-1*(deg2rad(110)) < diffOrientation) && (diffOrientation < deg2rad(110)))
            % Leader driving forward
            direction = 1;
            desiredOrientation = leaderOrientation;
            
            
        else
            % Leader driving reverse
            direction = -1;
            
            % Maintain forward heading but drive reverse
            if leaderOrientation <= 0
                desiredOrientation = leaderOrientation + pi;

            else
                desiredOrientation = leaderOrientation - pi;

            end
                        
        end

    else
        % Leader robot has stopped and maintain previous leader orientation
        direction = 2;
        desiredOrientation = previousOrientation;
    end

end
