function [desiredOrientation, direction] = InterpolateLeaderOrientation(currentARGlobalPosition, previousARGlobalPosition, tbOrientation)
    
    distance = sqrt((currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X)^2 + (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y)^2 );
    
    diffGlobalPosition = [(currentARGlobalPosition.Position.X - previousARGlobalPosition.Position.X), (currentARGlobalPosition.Position.Y - previousARGlobalPosition.Position.Y), (currentARGlobalPosition.Position.Z - previousARGlobalPosition.Position.Z)];

    % Determine if robot has moved or not
    if (distance > 0.01)
        leaderOrientation = atan2(diffGlobalPosition(2), diffGlobalPosition(1))

        diffOrientation = leaderOrientation - tbOrientation
               

        if ((-pi/2 < diffOrientation) && (diffOrientation < pi/2))
            % Leader driving forward
            direction = 1;
            desiredOrientation = leaderOrientation;

        else
            % Leader driving reverse
            direction = 0;
            
            % Maintain current heading but drive backwards
            if leaderOrientation <= 0
                desiredOrientation = leaderOrientation + pi;

            else
                desiredOrientation = leaderOrientation - pi;

            end

        end

    else
        % Leader robot has stopped and maintain previous leader orientation
        direction = 2;
        desiredOrientation = 0;
    end



end
