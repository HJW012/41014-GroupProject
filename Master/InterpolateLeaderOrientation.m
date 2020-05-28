function [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation, previousOrientation, previousDirection)
    
    distance = sqrt((currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X)^2 + (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y)^2 );
    
    diffGlobalPosition = [(currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X), (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y), (currentARGlobalPosition.Position.Z - previousARGlobalPosition.Position.Z)];

    % Determine if robot has moved or not
    if (distance > 0.05)
        leaderOrientation = atan2(diffGlobalPosition(2), diffGlobalPosition(1))

        diffOrientation = leaderOrientation - tbOrientation
        %diffOrientation = leaderOrientation - previousOrientation     

        if ((-1*(deg2rad(110)) < diffOrientation) && (diffOrientation < deg2rad(110)))
            % Leader driving forward
            direction = 1;
            desiredOrientation = leaderOrientation;
            
%             if previousDirection == 2
%                 direction = 1;
%             else
%                 direction = previousDirection;
%             end
            
        else
            % Leader driving reverse
            direction = -1;
            
            if leaderOrientation <= 0
                desiredOrientation = leaderOrientation + pi;

            else
                desiredOrientation = leaderOrientation - pi;

            end
%             if previousDirection == 2
%                 direction = -1;
%             else
%                 direction = -1 * previousDirection;
%             end
                        
        end

    else
        % Leader robot has stopped and maintain previous leader orientation
        direction = 2;
        desiredOrientation = previousOrientation;
    end

end
