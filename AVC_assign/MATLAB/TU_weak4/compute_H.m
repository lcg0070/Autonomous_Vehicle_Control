clear all; close all; clc;

syms cPsi sPsi cThe sThe cPhi sPhi real;
syms dotPsi dotThe dotPhi real;

rx = [1       0        0;
      0       cPhi     sPhi;
      0      -sPhi     cPhi];

ry = [cThe    0        -sThe;
      0       1         0;
      sThe    0         cThe];

rz = [cPsi    sPsi      0;
     -sPsi    cPsi      0;
      0       0         1];

pretty(simplify([dotPhi ; 0 ; 0] + rx * [0 ; dotThe; 0] + rx * ry * [0; 0; dotPsi]));

H = [1  0      -sThe;
     0  cPhi    cThe * sPhi;
     0 -sPhi    cThe * cPhi];

invH = simplify(inv(H))

