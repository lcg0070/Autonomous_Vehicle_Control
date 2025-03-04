function Y = dcm(Phi, Theta, Psi)
%DCM 이 함수의 요약 설명 위치
%   자세한 설명 위치

cosPhi = cos(Phi); sinPhi = sin(Phi);
cosPsi = cos(Psi); sinPsi = sin(Psi);
cosThe = cos(Theta); sinThe = sin(Theta);


rx = [1         0          0;
      0         cosPhi   sinPhi;
      0        -sinPhi   cosPhi];

ry = [cosThe    0        -sinThe;
      0         1         0;
      sinThe    0         cosThe];

rz = [cosPsi    sinPsi    0;
     -sinPsi    cosPsi    0;
      0         0         1];

Y = rx * ry * rz;

end

