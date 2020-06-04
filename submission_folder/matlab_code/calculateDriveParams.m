%% Calculate Drive Parameters
% Function used to calculate the required linear and angular velocity of
% the follower turtlebot to drive towards the target location
function [linearVel, angularVel] = calculateDriveParams(currentPose, currentOrientation, targetGlobalPose, targetOrientation) 
    
    % Load in the variables passed to the function  
    x1 = currentPose.X;
    y1 = currentPose.Y;
    z1 = currentPose.Z;
    x2 = targetGlobalPose(1);
    y2 = targetGlobalPose(2);
    z2 = targetGlobalPose(3);
    
    % Calculate the distance between the follower robot and the target
    % location
    distanceToTarget = calculateDistance(x1, y1, z1, x2, y2, z2);


    % Initialise variables
    linearGap = 0.1; % desired maximum gap between follower and target location
    angleGap = 0.001; % variable used as a multiplier
    angularGain = 0.75; % gain value used to calculate angular velocity
    linearGain = 0.5; % gain value used to calculate linear velocity
    
    % Initialise velocities to be zero (stationary)
    angularVel = 0;
    linearVel = 0;

    % If the distance to the target location is still too far away
    if (distanceToTarget >= linearGap)
        % Calculate angle between follower position and target position
        theta = calculateAngle(x1, y1, x2, y2);
        delta = currentOrientation; % Currently in range of -pi to pi with zero being right x axis

        diffAngle = theta - delta;

        if ((-1*(deg2rad(110)) < diffAngle) && (diffAngle < deg2rad(110)))
            % Target position in front of follower
            direction = 1;

        else
            % Target position behind follower
            direction = -1;
            
            % If required to reverse, set higher gains to back away from
            % the leader quickly.
            angularGain = 1.5;
            linearGain = 2.5;

            % Maintain heading in line with the target point but drive backwards
            if theta <= 0
                theta = theta + pi;
            else
                theta = theta - pi;
            end
        end

        angleToGo = abs(delta(1) - theta);

        % Case 1 - target above turtle
        if (y1 < y2)
            sigma = -(pi - theta);
            if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                if (abs(angleToGo)> abs(angleGap))
                    % Rotate clockwise
                    angularVel = -angularGain * abs(angleToGo);
                end
            end

            if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                if (abs(angleToGo) > abs(angleGap))
                    % Rotate counter-clockwise
                    angularVel = angularGain * abs(angleToGo);
                end
            end
        end

        % Case 2 - target below turtle
        if (y2 < y1)
            sigma = pi + theta;

            if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                if (abs(angleToGo) > abs(angleGap))
                    % Rotate clockwise
                    angularVel = -angularGain * abs(angleToGo);
                end
            end

            if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                if (abs(angleToGo) > abs(angleGap))
                    % Rotate counter-clockwise
                    angularVel = angularGain * abs(angleToGo);
                end
            end
        end

        % If reversing
        if direction < 0
            % If the angle between the turtlebot orientation and pointing
            % towards the target location is sufficient back away
            if (abs(angleToGo) <= 700 * angleGap)
                linearVel = direction * linearGain * (distanceToTarget + 0.3);
            end
        else
            % If the angle between the turtlebot orientation and pointing
            % towards the target location is sufficient drive towards
            % target
            if (abs(angleToGo) <= 500 * angleGap)
                linearVel = direction * linearGain * (distanceToTarget + 0.3);
            end

        end

    else
        % If distance between follower and target location is less than
        % 0.1m
        disp("Target Reached");

        theta = targetOrientation;
        delta = currentOrientation;

        angleToGo = theta - delta;

        % If the follower robot is 0.5 m behind the leader and facing the
        % correct orientation stop moving
        if (abs(angleToGo) <= 200 * angleGap)
            disp('Stopped Moving');
            linearVel = 0;
            angularVel = 0;

        else
            % Otherwise align with the leader robot
            
            % Stop driving forward/reverse
            linearVel = 0;

            % Case 1 - target above turtle
            if (y1 < y2)
                sigma = -(pi - theta);
                if (delta(1) > theta && delta(1) < pi || delta(1) > -pi && delta(1) < sigma)
                    if (abs(angleToGo)> abs(angleGap))
                        % Rotate clockwise
                        angularVel = -angularGain * abs(angleToGo);
                    end
                end

                if (delta(1) > sigma && delta(1) < 0 || delta(1) > 0 && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) 
                        % Rotate counter-clockwise
                        angularVel = angularGain * abs(angleToGo);
                    end
                end
            end

            % Case 2 - target below turtle
            if (y2 < y1)
                sigma = pi + theta;

                if (delta(1) < 0 && delta(1) > theta || delta(1) > 0 && delta(1) < sigma)
                    if (abs(angleToGo) > abs(angleGap)) 
                        % Rotate clockwise
                        angularVel = -angularGain * abs(angleToGo);
                    end
                end

                if (delta(1) > sigma && delta(1) < pi || delta(1) > -pi && delta(1) < theta)
                    if (abs(angleToGo) > abs(angleGap)) 
                        % Rotate counter-clockwise
                        angularVel = angularGain * abs(angleToGo);
                    end
                end
            end                
        end
    end
end

