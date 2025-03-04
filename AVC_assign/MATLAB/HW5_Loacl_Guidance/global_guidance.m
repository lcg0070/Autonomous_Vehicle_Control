function [X, Y, gamma, psi] = global_guidance(X_pro, Y_pro, gamma_pro)
%LOCAL_GUIDANCE 이 함수의 요약 설명 위치
%   자세한 설명 위치

global TIME k v x_t y_t;

v_x = v * cosd(gamma_pro);
v_y = v * sind(gamma_pro);

X = X_pro + v_x * TIME.SAMPLING;
Y = Y_pro + v_y * TIME.SAMPLING;

del_x = x_t - X;
del_y = y_t - Y;

r = sqrt(del_x^2 + del_y^2);
lambda = atan2d(del_y, del_x);

delta = gamma_pro - lambda;

gamma_dot = - k * v / r * sind(delta);

gamma = gamma_pro + gamma_dot * TIME.SAMPLING;

psi = gamma - lambda;

end

