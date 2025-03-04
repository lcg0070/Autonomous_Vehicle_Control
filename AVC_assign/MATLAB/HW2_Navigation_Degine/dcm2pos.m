function Pos = dcm2pos(DCM)
%DCM2POS 이 함수의 요약 설명 위치
%   자세한 설명 위치
arguments
    DCM (3, 3) double
end

theta = asin(-DCM(3, 1));

psi = atan2(DCM(2, 1), DCM(1, 1));

phi = atan2(DCM(3, 2), DCM(3, 3));

Pos = [psi, theta, phi];
end

