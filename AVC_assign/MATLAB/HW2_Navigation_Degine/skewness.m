function y = skewness(X)
%SKEWNESS 입력된 벡터로 왜대칭행렬(Skew-symmetric Matrix)를 만듭니다.
%   

arguments
    X (1, 3) double
end

y = [0     -X(3)   X(2);
     X(3)   0     -X(1);
    -X(2)   X(1)   0];

end

