function Uout = deriv_euler(Uin)
%DERIV_EULER 이 함수의 요약 설명 위치
%   Output
% Uout = [dotPhi ; dotThe ; dotPsi]

arguments
    Uin double
end

Phi = Uin(1);   % [rad] roll
The = Uin(2);   % [rad] ptich
Psi = Uin(3);   % [rad] yaw

Pnb = Uin(4);   % [rad/s] Wnb(ENUM.P)
Qnb = Uin(5);   % [rad/s] Wnb(ENUM.Q)
Rnb = Uin(6);   % [rad/s] Wnb(ENUM.R)

sPhi = sin(Phi); cPhi = cos(Phi);
tThe = tan(The); cThe = cos(The);

invH = [1 sPhi * tThe  cPhi * tThe;
        0 cPhi        -sPhi;
        0 sPhi / cThe  cPhi / cThe];

Uout = invH * [Pnb ; Qnb ; Rnb];

end

