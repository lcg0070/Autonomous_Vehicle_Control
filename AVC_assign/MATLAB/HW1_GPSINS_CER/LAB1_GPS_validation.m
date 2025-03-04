clear all; close all; clc;

% wgs84 = wgs84Ellipsoid;

% Centor
lat0 = 36.103455;
lon0 = 129.389652;
h0 = 57;

% Left
% lat0 = 36.103481;
% lon0 = 129.389555;
% h0 = 57;

% Right
% lat0 = 36.103429;
% lon0 = 129.389747;
% h0 = 57;

data = load("data_240319_192000.mat");

%=========================================================================%
% Plotting
%=========================================================================%

figure(1), clf, hold on, grid on;
plot(data.bfTime,data.bfTimeDel,'k','LineWidth',2);
ylim([0 1.4*data.TIME.Ts]);
ylabel("windows TIME increment [sec]");
xlabel("TIME [s]");

figure(2), clf;
subplot(3,1,1), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfEuler(data.ENUM.PSI,:),'r','LineWidth',2);
ylabel("psi [deg]");
subplot(3,1,2), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfEuler(data.ENUM.THE,:),'g','LineWidth',2);
ylabel("theta [deg]");
subplot(3,1,3), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfEuler(data.ENUM.PHI,:),'b','LineWidth',2);
ylabel("phi [deg]");
xlabel("TIME [s]");

figure(3), clf;
subplot(3,1,1), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfWib(data.ENUM.P,:),'r','LineWidth',2);
ylabel("p [deg/s]");
subplot(3,1,2), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfWib(data.ENUM.Q,:),'g','LineWidth',2);
ylabel("q [deg/s]");
subplot(3,1,3), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfWib(data.ENUM.R,:),'b','LineWidth',2);
ylabel("r [deg/s]");
xlabel("TIME [s]");

figure(4), clf;
subplot(3,1,1), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfPos(data.ENUM.LAT,:),'r','LineWidth',2);
ylabel("Latitude [deg]");
subplot(3,1,2), hold on, grid on;
plot(data.bfTime,data.UNIT.RAD2DEG*data.bfPos(data.ENUM.LON,:),'g','LineWidth',2);
ylabel("Longitude [deg]");
subplot(3,1,3), hold on, grid on;
plot(data.bfTime,data.bfPos(data.ENUM.HGT,:),'b','LineWidth',2);
ylabel("Altitude [m]");
xlabel("TIME [s]");

figure(5), clf;
subplot(3,1,1), hold on, grid on;
plot(data.bfTime,data.bfVn(data.ENUM.N,:),'r','LineWidth',2);
ylabel("vn [m/s]");
subplot(3,1,2), hold on, grid on;
plot(data.bfTime,data.bfVn(data.ENUM.E,:),'g','LineWidth',2);
ylabel("ve [m/s]");
subplot(3,1,3), hold on, grid on;
plot(data.bfTime,data.bfVn(data.ENUM.D,:),'b','LineWidth',2);
ylabel("vd [m/s]");
xlabel("TIME [s]");

figure(6), clf;
subplot(3,1,1), hold on, grid on;
plot(data.bfTime,data.bfAb(data.ENUM.X,:),'r','LineWidth',2);
ylabel("ax [m/s2]");
subplot(3,1,2), hold on, grid on;
plot(data.bfTime,data.bfAb(data.ENUM.Y,:),'g','LineWidth',2);
ylabel("ay [m/s2]");
subplot(3,1,3), hold on, grid on;
plot(data.bfTime,data.bfAb(data.ENUM.Z,:),'b','LineWidth',2);
ylabel("az [m/s2]");
xlabel("TIME [s]");

figure(7), clf, hold on, grid on;
plot(data.bfTime,data.bfPre,'k','LineWidth',2);
ylabel("Pressure [Pa]");
xlabel("TIME [s]");

figure(8), clf, hold on, grid on;
plot(data.bfTime,data.bfTem,'k','LineWidth',2);
ylabel("Temperature [deg celcius]");
xlabel("TIME [s]");

figure(9), clf;
geoplot( lat0 , lon0 , 'r' , 'LineWidth',2 );
geoplot(data.UNIT.RAD2DEG*data.bfPos(data.ENUM.LAT,:), data.UNIT.RAD2DEG*data.bfPos(data.ENUM.LON,:), 'g', 'LineWidth', 2);
geobasemap('satellite');

%=========================================================================%
% Compute CEP
%=========================================================================%

x_mean = 0;
x_dev = 0;

y_mean = 0;
y_dev = 0;

% size = data.N_DATA;
size = 1500;

x = zeros(size, 1);
y = zeros(size, 1);

% lat0 = mean(rad2deg(data.bfPos(data.ENUM.LAT, :)));
% lon0 = mean(rad2deg(data.bfPos(data.ENUM.LON, :)));
% h0 = mean(data.bfPos(data.ENUM.HGT, :));

for i = 1:size
    [x(i), y(i), z] = geodetic2rect(rad2deg(data.bfPos(data.ENUM.LAT, 16864 + i)), ...
                                    rad2deg(data.bfPos(data.ENUM.LON, 16864 + i)), ...
                                    lat0, lon0, h0);
    x_mean = x_mean + x(i);
    y_mean = y_mean + y(i);
end

x_mean = x_mean / size;
y_mean = y_mean / size;

for i = 1:size
    x_dev = x_dev + (x_mean - x(i))^2;
    y_dev = y_dev + (y_mean - y(i))^2;
end

x_dev = x_dev / size
y_dev = y_dev / size

virtual_x_mean = 0;
virtual_y_mean = 0;

rho = x_dev^2 + y_dev^2 + 2 * (x_dev * virtual_x_mean^2 + y_dev * virtual_y_mean^2);
eta = x_dev + virtual_x_mean^2 + y_dev + virtual_y_mean^2;
beta = 1 - 2 * rho / (9 * eta^2);

R = sqrt(eta * beta^3)

count = 0;

for i = 1:size
    if R >= sqrt(x(i)^2 + y(i)^2)
        count = count + 1;
    end
end

x_mean
y_mean

theta = 0:0.01:2*pi;

figure(10); hold on; grid on; axis equal;
fontsize(12, "points");
plot(y, x, 'o', 'Color', 'b');
plot(R * cos(theta) + y_mean, R * sin(theta) + x_mean, 'r', 'LineWidth', 2);
plot(0, 0, 'o', 'Color', 'r', LineWidth=3);
xlabel("{\Delta}y [m]");
ylabel("{\Delta}x [m]");
