function [TotalFinaltime TotalFinalA1 TotalFinalV1 TotalFinalP1 TotalFinalA2 TotalFinalV2 TotalFinalP2]=Fun_Total_Pass_Planner_2Module(Control_Sampling_Time, Pass_Initial_Point1, Pass_Delta_Point1,Pass_Initial_Point2, Pass_Delta_Point2);

    %----------------------------------------------------------------------%
    %Cartesian Pass Planner Data
    CorNum = 6;
    NumberModule = 2;
    A(1) = 1000;        % mm/sec^2
    V0(1) = 0;          % mm/sec
    Vmax(1) = 200;      % mm/sec
    A(2) = 1000;        % mm/sec^2
    V0(2) = 0;          % mm/sec
    Vmax(2) = 200;      % mm/sec
    A(3) = 1000;        % mm/sec^2           
    V0(3) = 0;          % mm/sec
    Vmax(3) = 200;      % mm/sec
    A(4) = (pi/4)*5;        % rad/sec^2
    V0(4) = 0;          % rad/sec
    Vmax(4) = pi/4;      % rad/sec
    A(5) = (pi/4)*5;        % rad/sec^2
    V0(5) = 0;          % rad/sec
    Vmax(5) = pi/4;      % rad/sec
    A(6) = (pi/4)*5;        % rad/sec^2           
    V0(6) = 0;          % rad/sec
    Vmax(6) = pi/4;      % rad/sec

    [NumPass, Dumy]=size(Pass_Delta_Point2);

    for CheckI = 1:NumPass
        if CheckI == 1
            P01(1) = Pass_Initial_Point1(1);          % mm
            Pf1(1) = Pass_Initial_Point1(1) + Pass_Delta_Point1(CheckI,1);        % mm
            P01(2) = Pass_Initial_Point1(2);          % mm
            Pf1(2) = Pass_Initial_Point1(2) + Pass_Delta_Point1(CheckI,2);          % mm
            P01(3) = Pass_Initial_Point1(3);          % mm
            Pf1(3) = Pass_Initial_Point1(3) + Pass_Delta_Point1(CheckI,3);          % mm
            P01(4) = Pass_Initial_Point1(4);          % rad
            Pf1(4) = Pass_Initial_Point1(4) + Pass_Delta_Point1(CheckI,4);          % rad
            P01(5) = Pass_Initial_Point1(5);          % rad
            Pf1(5) = Pass_Initial_Point1(5) + Pass_Delta_Point1(CheckI,5);          % rad
            P01(6) = Pass_Initial_Point1(6);          % rad
            Pf1(6) = Pass_Initial_Point1(6) + Pass_Delta_Point1(CheckI,6);          % rad
            
            P02(1) = Pass_Initial_Point2(1);          % mm
            Pf2(1) = Pass_Initial_Point2(1) + Pass_Delta_Point2(CheckI,1);        % mm
            P02(2) = Pass_Initial_Point2(2);          % mm
            Pf2(2) = Pass_Initial_Point2(2) + Pass_Delta_Point2(CheckI,2);          % mm
            P02(3) = Pass_Initial_Point2(3);          % mm
            Pf2(3) = Pass_Initial_Point2(3) + Pass_Delta_Point2(CheckI,3);          % mm
            P02(4) = Pass_Initial_Point2(4);          % rad
            Pf2(4) = Pass_Initial_Point2(4) + Pass_Delta_Point2(CheckI,4);          % rad
            P02(5) = Pass_Initial_Point2(5);          % rad
            Pf2(5) = Pass_Initial_Point2(5) + Pass_Delta_Point2(CheckI,5);          % rad
            P02(6) = Pass_Initial_Point2(6);          % rad
            Pf2(6) = Pass_Initial_Point2(6) + Pass_Delta_Point2(CheckI,6);          % rad
        else
            P01(1) = OldPos1(1);          % mm
            Pf1(1) = OldPos1(1) + Pass_Delta_Point1(CheckI,1);        % mm
            P01(2) = OldPos1(2);          % mm
            Pf1(2) = OldPos1(2) + Pass_Delta_Point1(CheckI,2);          % mm
            P01(3) = OldPos1(3);          % mm
            Pf1(3) = OldPos1(3) + Pass_Delta_Point1(CheckI,3);          % mm
            P01(4) = OldPos1(4);          % rad
            Pf1(4) = OldPos1(4) + Pass_Delta_Point1(CheckI,4);          % rad
            P01(5) = OldPos1(5);          % rad
            Pf1(5) = OldPos1(5) + Pass_Delta_Point1(CheckI,5);          % rad
            P01(6) = OldPos1(6);          % rad
            Pf1(6) = OldPos1(6) + Pass_Delta_Point1(CheckI,6);          % rad

            P02(1) = OldPos2(1);          % mm
            Pf2(1) = OldPos2(1) + Pass_Delta_Point2(CheckI,1);        % mm
            P02(2) = OldPos2(2);          % mm
            Pf2(2) = OldPos2(2) + Pass_Delta_Point2(CheckI,2);          % mm
            P02(3) = OldPos2(3);          % mm
            Pf2(3) = OldPos2(3) + Pass_Delta_Point2(CheckI,3);          % mm
            P02(4) = OldPos2(4);          % rad
            Pf2(4) = OldPos2(4) + Pass_Delta_Point2(CheckI,4);          % rad
            P02(5) = OldPos2(5);          % rad
            Pf2(5) = OldPos2(5) + Pass_Delta_Point2(CheckI,5);          % rad
            P02(6) = OldPos2(6);          % rad
            Pf2(6) = OldPos2(6) + Pass_Delta_Point2(CheckI,6);          % rad

        end
        OldPos1 = Pf1;
        OldPos2 = Pf2;
        
        A = [A A];
        V0 = [V0 V0];
        Vmax = [Vmax Vmax];
        
        P0 = [P01 P02];
        Pf = [Pf1 Pf2];
        
                
        [Finaltime FinalA FinalV FinalP] = Fun_Trapezoidal_Pass_Planner_2Module(CorNum*NumberModule,Control_Sampling_Time,A,V0,Vmax,P0,Pf);

        %----------------------------------------------------------------------%
        if CheckI == 1
            TotalFinalA = FinalA;
            TotalFinalV = FinalV;
            TotalFinalP = FinalP;
        else
            TotalFinalA = [TotalFinalA FinalA];
            TotalFinalV = [TotalFinalV FinalV];
            TotalFinalP = [TotalFinalP FinalP];
        end
    end
    
    [Dumy Count] = size(TotalFinalA);
    
    TotalFinaltime(1)=0;
    for CheckI=1:(Count-1)
        TotalFinaltime(CheckI+1) =  CheckI*Control_Sampling_Time;
    end
    
    for dumyI = 1:CorNum
     TotalFinalA1(dumyI,:) = TotalFinalA(dumyI,:);
     TotalFinalV1(dumyI,:) = TotalFinalV(dumyI,:);
     TotalFinalP1(dumyI,:) = TotalFinalP(dumyI,:);
    
     TotalFinalA2(dumyI,:) = TotalFinalA(dumyI+CorNum,:);
     TotalFinalV2(dumyI,:) = TotalFinalV(dumyI+CorNum,:);
     TotalFinalP2(dumyI,:) = TotalFinalP(dumyI+CorNum,:);
    end
   
    
end
