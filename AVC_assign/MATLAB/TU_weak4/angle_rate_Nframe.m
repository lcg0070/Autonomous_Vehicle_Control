function Uout = angle_rate_Nframe(Uin)
%ANGLE_RATE_NFRAME 이 함수의 요약 설명 위치
%   자세한 설명 위치
We  = Uin(1);   % [rad/s]
Re  = Uin(2);   % [m]
e2  = Uin(3);   % [-]
lat = Uin(4);   % [rad]
lon = Uin(5);   % [rad]
hgt = Uin(6);   % [m]
Vn  = Uin(7);   % [m/s]
Ve  = Uin(8);   % [m/s]
Vd  = Uin(9);   % [m/s]

sLat = sin(lat);

Rm = Re / sqrt(1 - e2 * sLat * sLat);
Rp = Re * (1 - e2) / (1 - e2 * sLat * sLat)^(1.5);

wie = [cos(lat) ; 0 ; -sin(lat)] * We;
wen = [ Ve / (Rp + hgt) ; -Vn / (Rm + hgt) ; -Ve * tan(lat) / (Rm + hgt)];

Uout = wie + wen;
end

