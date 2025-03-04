%==============================================================================%
% Autonomous Vehicle Navigation
% editted : 240327, v2
%==============================================================================%
clc; clear all; close all;

global FLAG ENUM UNIT TIME EARTH NAV

%============================= Loading Data ==============================%

DATA = readmatrix("data/[240405]Data.csv");

bf_euler_true = DATA(:,2:4);
bf_posN_true = DATA(:,[10, 11, 18]);
bf_vn_true = DATA(:,12:14);
bf_dotVb_true = DATA(:,15:17);
bf_wib_true = DATA(:,5:7);

bf_temp = DATA(:,19);
bf_atmos = DATA(:,20);

bf_time = (0:0.01:1300-0.01)';

%================================= Logic =================================%

FLAG = struct( 'NOK', false, 'YOK', true ) ;  

%============================== Enumeration ==============================%

ENUM = struct( 'X'  , 1, 'Y'  , 2, 'Z'  , 3, 'N_XYZ', 3,...
               'P'  , 1, 'Q'  , 2, 'R'  , 3, 'N_PQR', 3,...
               'PHI', 1, 'THE', 2, 'PSI', 3, 'N_ANG', 3,...
               'N'  , 1, 'E'  , 2, 'D'  , 3, 'N_NED', 3,...
               'LAT', 1, 'LON', 2, 'HGT', 3, 'N_POS', 3) ;

%============================ Unit Conversion ============================%

UNIT = struct( 'RAD2DEG', 180/pi , 'DEG2RAD', pi/180,...
               'FEET2METER', 0.3048, 'METER2FEET', 3.2808399);

%============================ Time Management ============================%

TIME = struct( 'count', 1   ,...    % [-]   count
               'sim'  , 0.00,...    % [sec] simulation time 
               'Ts'   , 0.01,...    % [sec] INS sampling time   
               'Start', 0.00,...    % [sec] simulation start time 
               'Final', 1299.99,... % [sec] simulation end   time
               'N'    , 0 );        % [-]   length of time sequence
TIME.N = length( TIME.Start:TIME.Ts:TIME.Final );

%============================== Earth Model ==============================%

EARTH = struct( 'Req' ,   6378137,...        % [m]     Radius of Earth 
                'Rn'  ,   0.0,...            % [m]     varies w.r.t Lattitude 
                'Re'  ,   0.0,...            % [m]     "     "      "     
                'We'  ,   7.292115e-5,...    % [rad/s] Turn rate of Earth 
                'g0'  ,   9.780318,...       % [m/s^2] Gravitational acceleration 
                'e2'  ,   6.69437999014e-3); % [-]     Eccentricity^2

%=============================== Navigation ==============================%

NAV = struct( 'refLat'      , 36  * UNIT.DEG2RAD,...    % [rad] reference latitude
              'refLon'      , 130 * UNIT.DEG2RAD,...    % [rad] reference longitude
              'refH'        , 100.0,...                 % [m]  reference height
              'vecPosN'     , zeros(ENUM.N_POS,1),...   % [rad]  geodetic position 
              'vecVn'       , zeros(ENUM.N_NED,1),... % [m/s]  velocity in N-frame
              'press'         , 0.00               ,... % [Pa]  barometric pressure
              'temp'         , 0.00               );   % [deg celsius] air temperature

%==================== Compute Haid: Value Initialize =====================%

bf_euler_true = bf_euler_true*UNIT.DEG2RAD;
bf_posN_true(:, 1:2) = bf_posN_true(:, 1:2) * UNIT.DEG2RAD;

M = 28.964917e-3;           % [kg/mol]      Molecular weight of air
R = 8.31446261815324;       % [N*m/(K*mol)] Gas constant

% US stardard atmosphere model
gama = -6.496062992126e-003;    % [K/m] Temperature per height
T0 = 288.15;                    % [K]   Temperature of the ice point
P0 = 1.01325e5;                 % [N*m]  See-level pressure
H0 = 0;                         % [m]    Initial height

% Cumpute simply term
density_scale = R / (EARTH.g0 * M);

flagStop = FLAG.NOK ; 

Hp = zeros(30000,1);

% Custom initalize
P0 = bf_atmos(1);
T0 = bf_temp(1) + 273.15;
H0 = bf_posN_true(1, ENUM.HGT);

%============================== Compute Haid =============================%
% Very small effect of the height to temperature.
% So, it assumes Lamda is 0.

while( flagStop == FLAG.NOK )

    pressure_ratio = bf_atmos(TIME.count) / P0;

    Hp(TIME.count) = H0 - T0 * density_scale * log(pressure_ratio);
%     Hp(TIME.count) =H0 + T0 * (pressure_ratio - density_scale * gama - 1)/ gama;

    % termination condition
    if( TIME.sim >= 299.99 )
        flagStop = FLAG.YOK ;
    end

    % Next Step
    TIME.sim   = TIME.Start + TIME.Ts * TIME.count;
    TIME.count = TIME.count + 1;
end

figure(1); hold on; grid on;
fontsize(1, 14, "point");
title("Barometer vs True Height");
plot(bf_time(1:30000), Hp, '--', 'Linewidth', 2);
plot(bf_time(1:30000), bf_posN_true(:, ENUM.HGT), 'Linewidth', 2);
xlabel("Time[s]");
ylabel("Height[m]");
legend("Barometer", "True");

%================= Filter simulation: Value Initialize ===================%

% tau = 4;
tau = 0.4;

ws2 = EARTH.g0 / EARTH.Req;
C1   = 3 / tau;
C2   = 2 * ws2 + 3 / tau^2;
C3   = 1 / tau^3;

h_init = bf_posN_true(1, ENUM.HGT);
vd_init = -bf_vn_true(1, ENUM.D);

haid.signals.values = cat(1, ones(100000, 1) * Hp(1), Hp);
haid.signals.dimensions = 1;
haid.time = bf_time;

ab.signals.values =  cat(1, ones(100000, 3) .* bf_dotVb_true(1, :), bf_dotVb_true);
ab.signals.dimensions = 3;
ab.time = bf_time;

pos.signals.values = cat(1, ones(100000, 2) .* bf_posN_true(1, 2:3), bf_posN_true(:, 2:3));
pos.signals.dimensions = 2;
pos.time = bf_time;

euler.signals.values = cat(1, ones(100000, 3) .* bf_euler_true(1, :), bf_euler_true);
euler.signlas.dimensions = 3;
euler.time = bf_time;

L.signals.values = cat(1, ones(100000, 1) * bf_posN_true(1, ENUM.LAT), bf_posN_true(:, ENUM.LAT));
L.signals.dimensions = 1;
L.time = bf_time;

vecPosN = [NAV.refLat;
           NAV.refLon;
           NAV.refH
          ];

out = sim("complementary_filter.slx");

%===================== Filter simulation: Plotting =======================%


figure(2); hold on; grid on;
fontsize(2, 14, "point");
plot(bf_time(1:30000), bf_posN_true(:, ENUM.HGT), 'Linewidth', 2);
plot(bf_time(1:30000), out.h(100001:end), 'Linewidth', 2);
xlabel("Time[s]");
ylabel("Height[m]");
% legend("Complement Filter", "True");

error = sqrt(mean((out.h(100001:end) - bf_posN_true(:, ENUM.HGT)).^2))

%%

h_init = 0;
vd_init = 0;

out = sim("complementary_filter.slx");

figure(2);
plot(bf_time(1:30000), out.h(100001:end), 'Linewidth', 2);

%%
% legend("True", "H_{hat}");
legend("True", "H_{hat} with H_aid", "H_{hat} with a_d");
