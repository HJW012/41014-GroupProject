function arGlobalPose = ConvertToGlobal(currentOffsetLocalPose, tbPose, tbOrientation)
    % Distance between TB and ARTag
    distance = abs(sqrt( (currentOffsetLocalPose.Position.Z)^2 + (currentOffsetLocalPose.Position.X)^2 ));

    % Convert to ARTag to global pose
    arGlobalPose.Position.Z = -currentOffsetLocalPose.Position.Y; %Not totally correct - dont take height of camera into account yet
    originX = tbPose.X;
    originY = tbPose.Y;
    zRot = tbOrientation(1);

    if (zRot > pi/2)
       if (currentOffsetLocalPose.Position.X < 0) % 1
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = zRot;
           gamma = pi - (alpha + theta);

           arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 1");
       elseif (currentOffsetLocalPose.Position.X > 0) % 2
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = zRot;
           gamma = pi - (theta - alpha);

           arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 2");
       end
    end

    if (zRot > 0 && zRot < pi/2)
        if (currentOffsetLocalPose.Position.X < 0) % 3
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = zRot;
           gamma = (alpha + theta);

           arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 3");
       elseif (currentOffsetLocalPose.Position.X > 0) % 4
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = zRot;
           gamma = (theta - alpha);

           arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 4");
       end
    end

    if (zRot < 0 && zRot > -pi/2)
        if (currentOffsetLocalPose.Position.X < 0) % 5
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = abs(zRot);
           gamma = (theta - alpha);

           arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 5");
       elseif (currentOffsetLocalPose.Position.X > 0) % 6
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = abs(zRot);
           gamma = (theta + alpha);

           arGlobalPose.Position.X = distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 6");
       end
    end

    if (zRot < -pi/2)
       if (currentOffsetLocalPose.Position.X < 0) % 7
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = abs(zRot);
           gamma = pi - (theta - alpha);

           arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 7");
       elseif (currentOffsetLocalPose.Position.X > 0) % 8
           alpha = atan(abs(currentOffsetLocalPose.Position.X / currentOffsetLocalPose.Position.Z));
           theta = abs(zRot);
           gamma = pi - (theta + alpha);

           arGlobalPose.Position.X = -distance * cos(abs(gamma)) + tbPose.X;
           arGlobalPose.Position.Y = -distance * sin(abs(gamma)) + tbPose.Y;
           disp("Case 8");
       end 
    end


end
