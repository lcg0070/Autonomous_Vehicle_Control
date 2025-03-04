function Wnb = compute_wnb(Cn2b, Wib, Vn, PosN)
%COMPUTE_WNB
%   자세한 설명 위치
arguments
    Cn2b (3, 3) double
    Wib (1, 3) double
    Vn (1, 3) double
    PosN (1, 3) double
end

L = PosN(1);
h = PosN(3);

Rm = 6478137 * (1 - 6.6943799014e-3) / (1 - 6.6943799014e-3 * sin(L)^2)^(3/2);
Rp = 6478137 / sqrt(1 - 6.6943799014e-3 * sin(L)^2);

nav_wen = [Vn(2) / (Rp + h);
           -Vn(1) / (Rm + h);
           -Vn(2) * tan(L) / (Rm + h)];

nav_wie = [cos(L) * 7.292115e-5;
           0;
           -sin(L) * 7.292115e-5];

nav_win = Cn2b * (nav_wie + nav_wen);

Wnb = Wib' - nav_win;

end

