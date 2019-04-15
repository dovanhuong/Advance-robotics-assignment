function ResultForward = Fun_Foward(Input_Th)

NumAxis = 6;

Link1 = 127.3; 
Link2 = 612.0;
Link3 = 572.3;
Link4 = 163.941;
Link5 = 115.7;
Link6 = 92.2; 

DH_a(1) =  0;
DH_alpha(1) = 0;
DH_d(1) =  Link1;
DH_theta(1) =  Input_Th(1);

DH_a(2) =    0;
DH_alpha(2) =  pi/2;
DH_d(2) =  0;
DH_theta(2) =  Input_Th(2) + pi/2;

DH_a(3) =   Link2;
DH_alpha(3) =  0;
DH_d(3) =  0;
DH_theta(3) =  Input_Th(3);

DH_a(4) =  Link3;
DH_alpha(4) =  0;
DH_d(4) =  Link4;
DH_theta(4) =  Input_Th(4) - pi/2;

DH_a(5) =  0;
DH_alpha(5) =  -pi/2;
DH_d(5) =  Link5;
DH_theta(5) =  Input_Th(5);

DH_a(6) =  0;
DH_alpha(6) =  pi/2;
DH_d(6) =  Link6;
DH_theta(6) =  Input_Th(6);
%-------------------------------------------------------%
% Solve TranMatrix

DumyDHTrans = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];

for Checki=1:NumAxis
    Mat1=[1 0 0 DH_a(Checki); 0 1 0 0;0 0 1 0;0 0 0 1];
    Mat2=[1 0 0 0; 0 cos(DH_alpha(Checki)) -sin(DH_alpha(Checki)) 0;0 sin(DH_alpha(Checki)) cos(DH_alpha(Checki)) 0;0 0 0 1];
    Mat3=[1 0 0 0; 0 1 0 0;0 0 1 DH_d(Checki);0 0 0 1];
    Mat4=[cos(DH_theta(Checki)) -sin(DH_theta(Checki)) 0 0; sin(DH_theta(Checki)) cos(DH_theta(Checki)) 0 0;0 0 1 0;0 0 0 1];
    
    DumyDHTrans= DumyDHTrans * Mat1 * Mat2 * Mat3 * Mat4;
    
    FinalDHTrans(:,:,Checki) = DumyDHTrans;

end
%-------------------------------------------------------%

PosintionEnd=FinalDHTrans(1:3,4,NumAxis);
FinalRotMat = FinalDHTrans(1:3,1:3,NumAxis);
EulerPi = atan2(FinalRotMat(2,3),FinalRotMat(1,3));
EulerTheta = atan2(sqrt(FinalRotMat(1,3)^2 +FinalRotMat(2,3)^2 ),FinalRotMat(3,3));
EulerFusai = atan2(FinalRotMat(3,2),-FinalRotMat(3,1));

ResultForward = [PosintionEnd;EulerPi;EulerTheta; EulerFusai];
