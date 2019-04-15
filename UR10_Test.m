clear all
close all
clc

InitialAngle=[0 -45*pi/180 pi/2 0 45*pi/180 pi/4];
ResultForward1=Fun_Foward(InitialAngle)
M_Zoff = 200;
%------------------------------------------------------------------------%
% Position of Module Initial
FG_FM1_X = 0;
FG_FM1_Y = 0;
FG_FM1_Z = 0;
FG_FM1_Pi = 0;
FG_FM1_Theta = 0.001;
FG_FM1_Fusai = 0;

FG_FM2_X = 0;
FG_FM2_Y = ResultForward1(1);
%FG_FM2_Y = 200;
FG_FM2_Z = 0;
FG_FM2_Pi = 0;
FG_FM2_Theta = 0.001;
FG_FM2_Fusai = 0;
%------------------------------------------------------------------------%
%Path Planning
Control_Sampling_Time = 0.02; %sec
Pass_Initial_Point1=[0, 0, 0, 0, 0, 0];
Pass_Delta_Point1  =[0, 0, 0, 0, 0, 0;
                     -100 -100 0 0 0 0;
                     0 0 0 0 0 0;
                     0 0 0 0 0 0;
                     0 0 0 0 0 0;
                     0 0 0 0 0 0;
                     0 0 0 0 0 pi/2;
                     1000 0 0 0 0 0;
                     0 0 0 0 0 0;
                     0 0 0 0 0 0;
                   ];

Pass_Initial_Point2=[0 0 0 0 0 0];
Pass_Delta_Point2  =[0 0 0 0 0 0;
                     -100 -100 0 0 0 0;
                     20 0 0 0 0 0;
                     100 0 -500 0 0 0;
                     20 0 0 0 0 0;
                     100 0 0 0 0 0;
                     0 0 0 0 0 pi/2;
                     1000 0 0 0 0 0;
                     -100 50 50 pi/4 0 pi/2;
                     -400 200 -500 0 0 0;
                     
                     ];
                    

[Finaltime FinalA1 FinalV1 FinalP1 FinalA2 FinalV2 FinalP2]=Fun_Total_Pass_Planner_2Module(Control_Sampling_Time, Pass_Initial_Point1, Pass_Delta_Point1, Pass_Initial_Point2, Pass_Delta_Point2);
[MatrixM,MatrixN] = size(FinalV1);

%----------------------------------------------------------------------%

%-------------------------------------------------------%
%Solve. Position of Endeffector and Jacobian



% Preallocate movie structure.
Fmov(1:MatrixN) = struct ( 'cdata', [], ...
                        'colormap', []);
                    
CheckF = figure('position', [200, 200, 600, 400]);                    


for Checki=1:MatrixN
    hold off
        
    %Solve Vel.of End Effector from Vel. of Module
    %Solve Vel of Module Frame
    FM1_FM1_PVel = [FinalV1(1,Checki);FinalV1(2,Checki);FinalV1(3,Checki)];
    FM1_FM1_WVel = [FinalV1(4,Checki);FinalV1(5,Checki);FinalV1(6,Checki)];
    FM2_FM2_PVel = [FinalV2(1,Checki);FinalV2(2,Checki);FinalV2(3,Checki)];
    FM2_FM2_WVel = [FinalV2(4,Checki);FinalV2(5,Checki);FinalV2(6,Checki)];
    
    %Solve Vel of Global Frame
    FG_FM2_Rot = Fun_RotMatrixEulerZYZ(FG_FM2_Pi,FG_FM2_Theta,FG_FM2_Fusai);
    FG_FM2_PVel = FG_FM2_Rot * FM2_FM2_PVel;
    FG_FM2_WVel = FG_FM2_Rot * FM2_FM2_WVel;
    FG_FM1_Rot = Fun_RotMatrixEulerZYZ(FG_FM1_Pi,FG_FM1_Theta,FG_FM1_Fusai);
    FG_FM1_PVel = FG_FM1_Rot * FM1_FM1_PVel;
    FG_FM1_WVel = FG_FM1_Rot * FM1_FM1_WVel;
    
    %Solve Arm Base Vel of Global Frame
    FG_FB2_WVel = FG_FM2_WVel;
    FG_FB2_PVel = FG_FM2_PVel + cross(FG_FM2_WVel,FG_FM2_Rot*[0;0;M_Zoff]);
    FG_FB1_WVel = FG_FM1_WVel;
    FG_FB1_PVel = FG_FM1_PVel + cross(FG_FM1_WVel,FG_FM1_Rot*[0;0;M_Zoff]);
            
    FG_FM1_Trans = Fun_TransMatrixEulerZYZ(FG_FM1_X,FG_FM1_Y,FG_FM1_Z,FG_FM1_Pi,FG_FM1_Theta,FG_FM1_Fusai);
    FM1_FB1_Trans = Fun_TransMatrixEulerZYZ(0,0,M_Zoff,pi/2.0,0,0);
    FG_FB1_Trans = FG_FM1_Trans * FM1_FB1_Trans;
    FG_FB1_Pos = FG_FB1_Trans(1:3,4);
    FG_FB1_Rot = FG_FB1_Trans(1:3,1:3);
  
    FG_FM2_Trans = Fun_TransMatrixEulerZYZ(FG_FM2_X,FG_FM2_Y,FG_FM2_Z,FG_FM2_Pi,FG_FM2_Theta,FG_FM2_Fusai);
    FM2_FB2_Trans = Fun_TransMatrixEulerZYZ(0,0,M_Zoff,-pi/2.0,pi,0);
    FG_FB2_Trans = FG_FM2_Trans * FM2_FB2_Trans;
    FG_FB2_Pos = FG_FB2_Trans(1:3,4);
    FG_FB2_Rot = FG_FB2_Trans(1:3,1:3);
    
    FB1_FB2_WVel = inv(FG_FB1_Rot) * (FG_FB2_WVel - FG_FB1_WVel);
    FB1_FB2_PVel = inv(FG_FB1_Rot) * ((FG_FB2_PVel - FG_FB1_PVel) - cross(FG_FB1_WVel,(FG_FB2_Pos - FG_FB1_Pos)));
    
    % 구해진 상대속도로 Jacobian 계산 및 위치 갱신
    [FB1_xvect,FB1_yvect,FB1_zvect,Jacobian]=ARM_Parm_Set(InitialAngle);
    InputCarVel = [FB1_FB2_PVel(1);FB1_FB2_PVel(2);FB1_FB2_PVel(3);FB1_FB2_WVel(1);FB1_FB2_WVel(2);FB1_FB2_WVel(3)];
    Vel = inv(Jacobian)*InputCarVel;
    InitialAngle(1) = InitialAngle(1) + Vel(1) *Control_Sampling_Time;
    InitialAngle(2) = InitialAngle(2) + Vel(2) *Control_Sampling_Time;
    InitialAngle(3) = InitialAngle(3) + Vel(3) *Control_Sampling_Time;
    InitialAngle(4) = InitialAngle(4) + Vel(4) *Control_Sampling_Time;
    InitialAngle(5) = InitialAngle(5) + Vel(5) *Control_Sampling_Time;
    InitialAngle(6) = InitialAngle(6) + Vel(6) *Control_Sampling_Time;
    
    %---------------------------------------------------------------------%
    %Update position of module 1
    %Update Position of Module1
     FG_FM1_X = FG_FM1_X + FG_FM1_PVel(1)*Control_Sampling_Time;
     FG_FM1_Y = FG_FM1_Y + FG_FM1_PVel(2)*Control_Sampling_Time;
     FG_FM1_Z = FG_FM1_Z + FG_FM1_PVel(3)*Control_Sampling_Time;
    
    VectToVel =  [0 -sin(FG_FM1_Pi) cos(FG_FM1_Pi)*sin(FG_FM1_Theta);
                  0 cos(FG_FM1_Pi) sin(FG_FM1_Pi)*sin(FG_FM1_Theta);
                  1         0      cos(FG_FM1_Theta)];
                 
    ResultDumy = inv(VectToVel) * [FG_FM1_WVel(1);FG_FM1_WVel(2);FG_FM1_WVel(3)];
    FG_FM1_Pi = FG_FM1_Pi + ResultDumy(1) * Control_Sampling_Time;
    FG_FM1_Theta = FG_FM1_Theta + ResultDumy(2) * Control_Sampling_Time;
    FG_FM1_Fusai = FG_FM1_Fusai + ResultDumy(3) * Control_Sampling_Time;
    %---------------------------------------------------------------------%
    
    %---------------------------------------------------------------------%
    %Update Position of Module2
     % update position of module 2
     FG_FM2_X = FG_FM2_X + FG_FM2_PVel(1)*Control_Sampling_Time;
     FG_FM2_Y = FG_FM2_Y + FG_FM2_PVel(2)*Control_Sampling_Time;
     FG_FM2_Z = FG_FM2_Z + FG_FM2_PVel(3)*Control_Sampling_Time;
    
    VectToVel =  [0 -sin(FG_FM2_Pi) cos(FG_FM2_Pi)*sin(FG_FM2_Theta);
                  0 cos(FG_FM2_Pi) sin(FG_FM2_Pi)*sin(FG_FM2_Theta);
                  1         0      cos(FG_FM2_Theta)];
            
              
                 
    ResultDumy = inv(VectToVel) * [FG_FM2_WVel(1);FG_FM2_WVel(2);FG_FM2_WVel(3)];
    FG_FM2_Pi = FG_FM2_Pi + ResultDumy(1) * Control_Sampling_Time
    FG_FM2_Theta = FG_FM2_Theta + ResultDumy(2) * Control_Sampling_Time
    FG_FM2_Fusai = FG_FM2_Fusai + ResultDumy(3) * Control_Sampling_Time
    %---------------------------------------------------------------------%

    %Plot Arm
    [FG_xvect,FG_yvect,FG_zvect]=Fun_Vec_Tran(FB1_xvect,FB1_yvect,FB1_zvect,FG_FB1_Trans);
    plot3(FG_xvect,FG_yvect,FG_zvect,'g','LineWidth',3);
    title('Robot UR10 mobile platform')
    hold on

    %Plot Module1
    M_Xoff = 100;
    xvect = [-M_Xoff*2, M_Xoff*2, M_Xoff*2, -M_Xoff*2,-M_Xoff*2];
    yvect = [-M_Xoff,    -M_Xoff, M_Xoff,      M_Xoff,-M_Xoff];
    zvect = [0,0,0,0,0];
    xvect = [xvect,xvect];
    yvect = [yvect,yvect];
    zvect = [0,0,0,0,0,M_Zoff,M_Zoff,M_Zoff,M_Zoff,M_Zoff];
    xvect = [xvect,nan,M_Xoff*2,M_Xoff*2,nan,M_Xoff*2,M_Xoff*2,nan,-M_Xoff*2,-M_Xoff*2];
    yvect = [yvect,nan,-M_Xoff,-M_Xoff,nan,M_Xoff,M_Xoff,nan,M_Xoff,M_Xoff];
    zvect = [zvect,nan,0,M_Zoff,nan,0,M_Zoff,nan,0,M_Zoff];
    [Module1_xvect,Module1_yvect,Module1_zvect]=Fun_Vec_Tran(xvect,yvect,zvect,FG_FM1_Trans);
    plot3(Module1_xvect,Module1_yvect,Module1_zvect,'b','LineWidth',2)
    
    %Plot Module2
    %M_Xoff = 50;
    %xvect = [-M_Xoff*2, M_Xoff*2, M_Xoff*2, -M_Xoff*2,-M_Xoff*2];
    %yvect = [-M_Xoff,    -M_Xoff, M_Xoff,      M_Xoff,-M_Xoff];
    %zvect = [0,0,0,0,0];
    %xvect = [xvect,xvect];
    %yvect = [yvect,yvect];
    %zvect = [0,0,0,0,0,M_Zoff,M_Zoff,M_Zoff,M_Zoff,M_Zoff];
    %xvect = [xvect,nan,M_Xoff*2,M_Xoff*2,nan,M_Xoff*2,M_Xoff*2,nan,-M_Xoff*2,-M_Xoff*2];
    %yvect = [yvect,nan,-M_Xoff,-M_Xoff,nan,M_Xoff,M_Xoff,nan,M_Xoff,M_Xoff];
    %zvect = [zvect,nan,0,M_Zoff,nan,0,M_Zoff,nan,0,M_Zoff];
    %[Module2_xvect,Module2_yvect,Module2_zvect]=Fun_Vec_Tran(xvect,yvect,zvect,FG_FM2_Trans);
    %plot3(Module2_xvect,Module2_yvect,Module2_zvect,'g','LineWidth',2)
        
    xlabel('x');
    ylabel('y');
    zlabel('z');
    axis([-500 2000 -500 2000 -500 2000]);
    view([-1 1 1 ]);
    grid
        
    movegui(CheckF)
    Fmov(Checki) = getframe(CheckF);
    
    pause(Control_Sampling_Time);
    
    
    hold on
end
%-------------------------------------------------------%

movie2avi(Fmov,'Main.avi','compression','none');

