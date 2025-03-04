clear all; clc;

%============================ Value Initalize ============================%

time = 0:1e-3:1000;

w0 = 0.5;

tau = 1;
taus = 0.01:0.01:2;

bias = 0.1;

ws2 = 9.780318 / 6378137;

baro_freq = 1;  % [Hz]
imu_freq = 100;  % [Hz]

freq = 1:1:5;
optimation_tau = zeros(length(freq), 1);
errors = zeros(length(taus), 1);

idx = 1;

for baro_freq = freq
    w0_rad = 2 * pi * w0;

    min_error_tau = 0;
    min_error = Inf;

    idx_error = 1;

    for tau = taus
        C1 = 3 / tau;
        C2 = 2 * ws2 + 3 / tau^2;
        C3 = 1 / tau^3;

%========================== System Configuration =========================%

        barometer = tf(2 * pi * baro_freq, [1, 2 * pi * baro_freq]);
        imu = tf(2 * pi * imu_freq, [1, 2 * pi * imu_freq]);

        h_aid_transfer = tf([C1, C2, C3], [1, C1, C2 - 2 * ws2, C3]);
        ad_transfer = -tf([1, 0], [1, C1, C2 - 2 * ws2, C3]);

%=============================== Simulation ==============================%

        h_true = sin(w0_rad * time)';
        a_true = -w0_rad^2 * sin(w0_rad * time);

        h_aid = lsim(barometer, h_true, time);
        ad = lsim(imu, a_true, time) + bias;

        h_hat_aid = lsim(h_aid_transfer, h_aid, time);
        h_hat_ad = lsim(ad_transfer, ad, time);
        h_hat = h_hat_ad + h_hat_aid;

        error = sqrt(mean((h_true(500000:end) - h_hat(500000:end)).^2));

        errors(idx_error) = error;

        idx_error = idx_error + 1;

%         figure(3);
%         plot(time, h_hat);

    end

    figure(2); hold on; grid on;

    plot(taus, errors, "LineWidth", 2);
%     plot(time, h_true);
%     plot(time, h_hat_aid);
%     plot(time, h_hat_ad);
%     plot(time, h_hat);

    optimation_tau(idx) = min_error_tau;
    idx = idx + 1;
end

%%
figure(1);
xlabel("\tau[sec]");
ylabel("RMS error[m]");
title("RMS error");
fontsize(1, 14, "point");
legend("\omega_{IMU} = 110[Hz]", "\omega_{IMU} = 120[Hz]", "\omega_{IMU} = 130[Hz]",...
       "\omega_{IMU} = 140[Hz]", "\omega_{IMU} = 150[Hz]");
ylim([0, 1.7]);
