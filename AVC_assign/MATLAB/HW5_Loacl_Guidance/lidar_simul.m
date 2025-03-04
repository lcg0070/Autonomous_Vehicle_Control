function [angles, ranges] = lidar_simul(objects)
%LIDAR_SIMUL 이 함수의 요약 설명 위치
%   자세한 설명 위치

global LIDAR;

angles = (-135 + LIDAR.HEADING):0.125:(135 + LIDAR.HEADING);
ranges = ones(1, length(angles)) * LIDAR.RANGE;

for object = objects
    center_range = sqrt((object(1)-LIDAR.X)^2 + (object(2)-LIDAR.Y)^2);
    min_range = center_range - object(3);

    if min_range > LIDAR.RANGE
        continue;
    end

    center_angle = atan2d(object(2) - LIDAR.Y, object(1) - LIDAR.X);
    del_angle = asind(object(3) / center_range);

    min_angle = center_angle - del_angle;
    max_angle = center_angle + del_angle;

    for i = 1:length(angles)
        if angles(i) < min_angle
            continue;
        elseif angles(i) > max_angle
            break;
        end

        ranges(i) = center_range - object(3) * cosd(pi * (center_angle - angles(i)));
    end
end

end

