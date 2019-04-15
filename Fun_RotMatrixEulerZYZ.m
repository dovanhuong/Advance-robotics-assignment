function Rot = Fun_RotMatrixEulerZYZ(pi, theta, fusai)

A = [cos(pi) -sin(pi) 0;sin(pi) cos(pi) 0;0 0 1];
B = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
C = [cos(fusai) -sin(fusai) 0;sin(fusai) cos(fusai) 0;0 0 1];

Rot = A*B*C;

[i,j]=size(Rot);
end