clc; clear all; close all;

files = dir("240415_data");

g0 = 9.780318;
M = 28.964917e-3;           % [kg/mol]      Molecular weight of air
R = 8.31446261815324;       % [N*m/(K*mol)] Gas constant

density_scale = R / (g0 * M);
baro_alt_mean = zeros(1, 751);

for i = 1:length(files)
    file = files(i);

    if(file.isdir)
        continue;
    end

    data = load("240415_data\" + file.name);

    bfPre = data.bfPre;
    bfTem = data.bfTem;
    bfTime = data.bfTime;

    P0 = bfPre(1);
    T0 = bfTem(1) + 273.15;
    H0 = 81;

    pressure_ratio = bfPre / P0;

    baro_alt = H0 - T0 * density_scale * log(pressure_ratio);

    baro_alt_mean = baro_alt_mean + baro_alt;
end

baro_alt_mean = baro_alt_mean / (length(files) - 2);

u = @(t) (t >= 0);    % Unit Step Funtion
system_1st = @(tau, time) (1 - exp(-1 / tau * time)) .* u(time);

system = @(x, time) -x(1) * system_1st(x(2), time - x(3)) + x(4);

x= lsqcurvefit(system, [2; 0.5; 10; H0], bfTime, baro_alt_mean);

time = bfTime(1):1e-3:bfTime(end);

yout = system(x, time);

figure(1); hold on;
fontsize(1, 16, 'point');
plot(bfTime, baro_alt_mean);
plot(time, yout, 'LineWidth', 2);
legend("Law Data", "Estimation System")
xlabel("Time[s]");
ylabel("Height[m]");

disp("Tau");
disp(x(2));


