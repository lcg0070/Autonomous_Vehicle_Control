clear all; close all; clc;

% ======================= Global value initailize ======================= %

V = 3;
k = 4;

L = 0.729321661910;

tims_sampling = 1e-3;

motor2steer_ratio = 33 / 60;

tau = 0.1;

% Point1
gamma0 = pi / 4;

x0 = 0;
y0 = 0;

xt = 10;
yt = 0;

out = sim("guidance.slx");

figure(1); hold on; grid on;
plot(out.x, out.y, "LineWidth", 2);

total_steer = out.gamma;
total_x = out.x;
total_y = out.y;
total_time = out.time;

% % Point2
% time0 = out.time(end);
% gamma0 = out.gamma(end);
% 
% x0 = out.x(end);
% y0 = out.y(end);
% 
% xt = -10;
% yt = -5;
% 
% out = sim("guidance.slx");
% 
% figure(1);
% plot(out.x, out.y, "LineWidth", 2);
% 
% total_steer = [total_steer; out.gamma];
% total_x = [total_x; out.x];
% total_y = [total_y; out.y];
% total_time = [total_time; time0 + out.time];
% 
% % Point3
% time0 = time0 + out.time(end);
% gamma0 = out.gamma(end);
% 
% x0 = out.x(end);
% y0 = out.y(end);
% 
% xt = 0;
% yt = 10;
% 
% out = sim("guidance.slx");
% 
% figure(1);
% plot(out.x, out.y, "LineWidth", 2);
% 
% total_steer = [total_steer; out.gamma];
% total_x = [total_x; out.x];
% total_y = [total_y; out.y];
% total_time = [total_time; time0 + out.time];
% 
% % Point4
% time0 = time0 + out.time(end);
% gamma0 = out.gamma(end);
% 
% x0 = out.x(end);
% y0 = out.y(end);
% 
% xt = 0;
% yt = 0;
% 
% out = sim("guidance.slx");
% 
% figure(1);
% plot(out.x, out.y, "LineWidth", 2);
% 
% 
% total_steer = [total_steer; out.gamma];
% total_x = [total_x; out.x];
% total_y = [total_y; out.y];
% total_time = [total_time; time0 + out.time];

% Plot Configuration
figure(1);
title("Position");
xlabel("x[m]");
ylabel('y[m]');

figure(2); hold on; grid on;
title("x-Posiotion");
plot(total_time, total_x, "LineWidth", 2);
xlabel('Time[s]');
ylabel('x[m]');

figure(3); hold on; grid on;
title("y-Posiotion");
plot(total_time, total_y, "LineWidth", 2);
xlabel('Time[s]');
ylabel('y[m]');

figure(4); hold on; grid on;
title("Distance");
plot(total_time, out.r, "LineWidth", 2);
xlabel('Time[s]');

figure(5); hold on; grid on;
title("\dot{\gamma} Command");
plot(total_time, out.gamma_command, "LineWidth", 2);
xlabel('Time[s]');
ylabel('y[m]');

figure(6); hold on; grid on;
title("\dot{\gamma}");
plot(total_time, out.gamma_dot, "LineWidth", 2);
xlabel('Time[s]');
ylabel('y[m]');

figure(7); hold on; grid on;
title('Yaw');
plot(total_time, out.gamma, "LineWidth", 2);
xlabel('Time[s]');
ylabel('\psi[rad]');
