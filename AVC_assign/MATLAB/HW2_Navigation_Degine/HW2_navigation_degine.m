clear all; close all; clc;

global FLAG ENUM UNIT TIME EARTH

%============================= Loading Data ==============================%
DATA = load("data\AVC_data.mat");

bf_euler_true = DATA.bfEulerTrue';
bf_posN_true = DATA.bfPosNTrue';
bf_dotVi_true = DATA.bfdotViTrue';
bf_vec_vn_true = DATA.bfvecVnTrue';
bf_wib_true = DATA.bfWibTrue';

%================================= Logic =================================%

FLAG = struct('NOK', false, 'YOK', true);

%============================== Enumeration ==============================%

ENUM = struct('x',   1, 'y',   2, 'z',   3, 'NUM_XYZ', 3,...
              'P',   1, 'Q',   2, 'R',   3, 'NUM_PQR', 3,...
              'PHI', 1, 'THE', 2, 'PSI', 3, 'NUM_ANG', 3,...
              'N',   1, 'E',   2, 'D',   3, 'NUM_NED', 3,...
              'LAT', 1, 'LON', 2, 'HGT', 3, 'NUM_POS', 3);

%============================ Unit Conversion ============================%

UNIT = struct('RAD2DEG', 180 / pi, 'DEG2RAD', pi / 180);

%============================ Time Management ============================%

TIME = struct('count',    1,...         % [-]   Count
              'sim',      0.,...        % [sec] Simulation time
              'sampling', 0.02,...      % [sec] Sampling time
              'start',    0.,...        % [sec] Simulation start time
              'final',    3599.98,...   % [sec] simulation end time
              'N',        0);           % [-]   Length of time sequence

TIME.N = length(TIME.start : TIME.sampling : TIME.final);

%============================== Earth Model ==============================%

EARTH = struct('Re',  6378137,...       % [m] Radius of the Earth
               'Rm',  0.,...            % [m] RN
               'Rp',  0.,...            % [m] RE
               'We',  7.292115e-5, ...  % [rad/s] Turn rate of Earth
               'g0',  9.780318,...      % [m/s^2] Gravity acceleration
               'e2',  6.6943799014e-3); % [-] Eccentricity^2

%=============================== Navigation ==============================%

NAV = struct('refLat',  36 * UNIT.DEG2RAD,...       % [rad] Referece lattitude
             'refLot',  130 * UNIT.DEG2RAD,...      % [rad] Referece longitude
             'refH',    100.,...                    % [m] Referece height
             'vecPosN', zeros(ENUM.NUM_POS, 1),...  % Geodetic Posiotion
             'vecVn',   zeros(ENUM.NUM_NED, 1),...  % Velocity in N-frame
             'pre',     0.,...                      % Pressure
             'tem',     0.);                        % Temperature

%=========================== Value Initialize ============================%

Cb2n_init = dcm(bf_euler_true(1, ENUM.PHI),...
                bf_euler_true(1, ENUM.THE),...
                bf_euler_true(1, ENUM.PSI))';

Vn_init = [bf_vn_true(1, ENUM.N);
           bf_vn_true(1, ENUM.E);
           bf_vn_true(1, ENUM.D)];

posN_init = [bf_posN_true(1, ENUM.LAT);
             bf_posN_true(1, ENUM.LON);
             bf_posN_true(1, ENUM.HGT)];

time = TIME.start : TIME.sampling : TIME.final;

simul_wib.time = time;
simul_wib.signals.values = bf_wib_true;
simul_wib.signals.dimensions = 3;

simul_vn.time = time;
simul_vn.signals.values = bf_vec_vn_true;
simul_vn.signals.dimensions = 3;

simul_PosN.time = time;
simul_PosN.signals.values = bf_posN_true;
simul_PosN.signals.dimensions = 3;

%=========================== Simulation & Plot ===========================%

out = sim("navigation_degine.slx");

figure(1); hold on; grid on;
plot(time, bf_euler_true(:, ENUM.PHI) * UNIT.RAD2DEG, "LineStyle","-");
plot(time, out.euler(:, ENUM.PHI) * UNIT.RAD2DEG, "LineStyle", "--");

figure(2); hold on; grid on;
plot(time, bf_euler_true(:, ENUM.THE) * UNIT.RAD2DEG, "LineStyle","-");
plot(time, out.euler(:, ENUM.THE) * UNIT.RAD2DEG, "LineStyle", "--");

figure(3); hold on; grid on;
plot(time, bf_euler_true(:, ENUM.PSI) * UNIT.RAD2DEG, "LineStyle","-");
plot(time, out.euler(:, ENUM.PSI) * UNIT.RAD2DEG, "LineStyle", "--");
