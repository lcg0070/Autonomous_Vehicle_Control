clear all; close all; clc;

global LIDAR TIME k v x_t y_t;

%===================== Comfiguration Global Value ========================%

LIDAR = struct('RANGE', 15,...      % [m]   Maximum range
               'RESOL', 0.125, ...  % [deg] LIDAR resolution
               'ANGLE', 270, ...    % [deg] LIDAR anlge range
               'X', 0, ...          % [m]   Currunt location X
               'Y', 0, ...          % [m]   Currunt location Y
               'HEADING', 90);       % [deg] Currunt heading

TIME = struct('START', 0,...
              'FINAL', 1000,...
              'SAMPLING', 0.01,...
              'TIME', 0);

TIME.TIME = TIME.START:TIME.SAMPLING:TIME.FINAL;

%===================== Comfiguration Local Value =========================%

objects = [5, 5, 2;...
           -1, 4, 1]';

theta = 0:(2*pi/1000):(2*pi);

x_t = 30;
y_t = 30;

k = 4;

v = 1;

sigma = 1;

e = 0.5;

fov = -90:0.125:90;

%============================= Simulation ================================%

for time = TIME.TIME

    [angles, ranges] = lidar_simul(objects);

    [x, y, gamma, psi] = global_guidance(LIDAR.X, LIDAR.Y, LIDAR.HEADING);

    sff = LIDAR.RANGE * exp((deg2rad(fov) - deg2rad(psi)).^2 / (2 * sigma^2));

    off = ranges(1, 361:end-360);

    iff = e * sff + (1 - e) * off;

    if(sqrt((x_t - x)^2 + (y_t - y)^2) <= 0.5)
        break;
    end

    [command, idx] = max(iff);

%============================ Draw Object ================================%

    object1_x = objects(3, 1) * cos(theta) + objects(1, 1);
    object1_y = objects(3, 1) * sin(theta) + objects(2, 1);

    object2_x = objects(3, 2) * cos(theta) + objects(1, 2);
    object2_y = objects(3, 2) * sin(theta) + objects(2, 2);

    lidar_x = ranges .* cosd(angles) + LIDAR.X;
    lidar_y = ranges .* sind(angles) + LIDAR.Y;

%============================ Data Update ================================%

    LIDAR.X = x;
    LIDAR.Y = y;
    LIDAR.HEADING = gamma;

%================================ Plot ===================================%

    if rem(time, 1) == 0
        figure(1); hold on; grid on; axis equal; drawnow;
        plot(object1_x, object1_y, 'r', 'LineWidth', 2);
        plot(object2_x, object2_y, 'r', 'LineWidth', 2);
        plot(lidar_x, lidar_y, 'b', 'LineWidth', 2);
        plot(LIDAR.X, LIDAR.Y, 'bo');

        figure(2);
        plot(fov, sff / LIDAR.RANGE);

        figure(3);
        plot(fov, off / LIDAR.RANGE);

        figure(4);
        plot(fov, iff / (2 * LIDAR.RANGE));

        clf(1);
    end

end

% figure(2);
% plot(angles, ranges);
