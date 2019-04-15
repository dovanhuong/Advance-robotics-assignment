function [Finaltime FinalA FinalV FinalP]=Fun_Trapezoidal_Pass_Planner_2Module(N,Control_Sampling_Time,A,V0,Vmax,P0,Pf);

for ANum =1:N
	Delta(ANum) = Pf(ANum) - P0(ANum);
	if Delta(ANum) < 0 
         Dir(ANum) = -1;
     else
        Dir(ANum) = 1;   
	end
	
    Ta0(ANum) = (Dir(ANum)*Vmax(ANum) - V0(ANum))/(Dir(ANum)*A(ANum));
	Taf(ANum) = Vmax(ANum)/A(ANum);
	
	DeltaXa0(ANum) = V0(ANum)*Ta0(ANum) + 0.5*Dir(ANum)*A(ANum)*(Ta0(ANum))^2;
	DeltaXaf(ANum) = Dir(ANum)*Vmax(ANum)*Taf(ANum) - 0.5*Dir(ANum)*A(ANum)*(Taf(ANum))^2;
	DeltaTsat(ANum) = (Pf(ANum)-(P0(ANum)+DeltaXa0(ANum)+DeltaXaf(ANum)))/(Dir(ANum)*Vmax(ANum));

    if DeltaTsat(ANum) < 0 
        Vmax(ANum) = sqrt( Dir(ANum)*A(ANum)*(Pf(ANum) - P0(ANum))+0.5*V0(ANum)^2);
        Ta0(ANum) = (Dir(ANum)*Vmax(ANum) - V0(ANum))/(Dir(ANum)*A(ANum));
        Taf(ANum) = Vmax(ANum)/A(ANum);
        DeltaXa0(ANum) = V0(ANum)*Ta0(ANum) + 0.5*Dir(ANum)*A(ANum)*(Ta0(ANum))^2;
        TotalTime(ANum) = Ta0(ANum) +  Taf(ANum);
        DeltaTsat(ANum) = 0;
    else
        TotalTime(ANum) = Ta0(ANum) +  Taf(ANum) + DeltaTsat(ANum);
    end
    
    T1(ANum) = Ta0(ANum);
    DeltaT1(ANum) = Ta0(ANum);
    T2(ANum) = Ta0(ANum)+DeltaTsat(ANum);
    DeltaT2(ANum) = DeltaTsat(ANum);
    T3(ANum) = Ta0(ANum)+DeltaTsat(ANum)+Taf(ANum);
    DeltaT3(ANum) = Taf(ANum);
end

MaxT = max(TotalTime);

for ANum =1:N
    
	DeltaA(ANum) = MaxT - (T3(ANum)-DeltaT2(ANum));
	Delta(ANum) = -DeltaA(ANum)/2.0 + sqrt((DeltaA(ANum)/2.0)^2   + (MaxT-T3(ANum))*abs(Vmax(ANum))/A(ANum)) ;
	
	T1(ANum) = T1(ANum) - Delta(ANum);
	T2(ANum) = MaxT-(DeltaT3(ANum)-Delta(ANum));
	T3(ANum) = MaxT;
	DeltaT1(ANum) = T1(ANum);
	DeltaT2(ANum) = T2(ANum)-T1(ANum);
	DeltaT3(ANum) = T3(ANum)-T2(ANum);
	Vmax(ANum) = abs(V0(ANum)+Dir(ANum)*A(ANum)*T1(ANum));
	DeltaXa0(ANum) = V0(ANum)*DeltaT1(ANum) + 0.5*Dir(ANum)*A(ANum)*(DeltaT1(ANum))^2;
	DeltaXaf(ANum) = Dir(ANum)*Vmax(ANum)*DeltaT3(ANum) - 0.5*Dir(ANum)*A(ANum)*(DeltaT3(ANum))^2;
	
	i = 0;

	if T1(ANum) < 0
        
        Tstop = abs(V0(ANum))/A(ANum);
        Xstop = 0.5*Dir(ANum)*A(ANum)*Tstop^2;
        Vmax(ANum) = abs(( (Pf2(ANum)-P02(ANum))-Xstop )/(MaxT-Tstop));
        
        T1(ANum) = abs(Dir(ANum)*Vmax(ANum)-V0(ANum))/A(ANum);
        T2(ANum) = MaxT - (Tstop-T1(ANum));
        T3(ANum) = MaxT;
        DeltaT1(ANum) = T1(ANum);
        DeltaT2(ANum) = T2(ANum)-T1(ANum);
        DeltaT3(ANum) = T3(ANum)-T2(ANum);
        
      	DeltaXa0(ANum) = V0(ANum)*DeltaT1(ANum) - 0.5*Dir(ANum)*A(ANum)*(DeltaT1(ANum))^2;
    	DeltaXaf(ANum) = Dir(ANum)*Vmax(ANum)*DeltaT3(ANum) - 0.5*Dir(ANum)*A(ANum)*(DeltaT3(ANum))^2;
    
         for time=0.0:Control_Sampling_Time:T3(ANum)
                i = i+1;
                Finaltime(ANum,i) = time;
                if time<T1(ANum)
                    FinalA(ANum,i) = -Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = V0(ANum) - Dir(ANum)*A(ANum)*time;
                    FinalP(ANum,i) = P02(ANum) + V0(ANum)*time - 0.5*Dir(ANum)*A(ANum)*(time)^2;
                elseif time< T2(ANum)
                    FinalA(ANum,i) = 0;
                    FinalV(ANum,i) = Dir(ANum)*Vmax(ANum);
                    FinalP(ANum,i) = P0(ANum) + DeltaXa0(ANum) + Dir(ANum)*Vmax(ANum)*(time-T1(ANum));
                else
                    FinalA(ANum,i) = -Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = Dir(ANum)*Vmax(ANum)-Dir(ANum)*A(ANum)*(time-T2(ANum));
                    FinalP(ANum,i) = P0(ANum) + DeltaXa0(ANum) + Dir(ANum)*Vmax(ANum)*(DeltaT2(ANum))+ Dir(ANum)*Vmax(ANum)*(time-T2(ANum)) -0.5*Dir(ANum)*A(ANum)*(time-T2(ANum))^2;
                end
            end
	else    
		if DeltaT2(ANum) == 0
               for time=0.0:Control_Sampling_Time:T3
                i = i+1;
                Finaltime(ANum,i) = time;
                if time<T1(ANum)
                    FinalA(ANum,i) = Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = V0(ANum) + Dir(ANum)*A(ANum)*time;
                    FinalP(ANum,i) = P0(ANum) + V0(ANum)*time + 0.5*Dir(ANum)*A(ANum)*(time)^2;
                else
                    FinalA(ANum,i) = -Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = Dir(ANum)*Vmax(ANum)-Dir(ANum)*A(ANum)*(time-T1(ANum));
                    FinalP(ANum,i) = P0(ANum) + DeltaXa0(ANum) + Dir(ANum)*Vmax(ANum)*(time-T1(ANum)) -0.5*Dir(ANum)*A(ANum)*(time-T1(ANum))^2;
                end
            end
         else
            for time=0.0:Control_Sampling_Time:T3(ANum)
                i = i+1;
                Finaltime(ANum,i) = time;
                if time<T1(ANum)
                    FinalA(ANum,i) = Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = V0(ANum) + Dir(ANum)*A(ANum)*time;
                    FinalP(ANum,i) = P0(ANum) + V0(ANum)*time + 0.5*Dir(ANum)*A(ANum)*(time)^2;
                elseif time< T2(ANum)
                    FinalA(ANum,i) = 0;
                    FinalV(ANum,i) = Dir(ANum)*Vmax(ANum);
                    FinalP(ANum,i) = P0(ANum) + DeltaXa0(ANum) + Dir(ANum)*Vmax(ANum)*(time-T1(ANum));
                else
                    FinalA(ANum,i) = -Dir(ANum)*A(ANum);
                    FinalV(ANum,i) = Dir(ANum)*Vmax(ANum)-Dir(ANum)*A(ANum)*(time-T2(ANum));
                    FinalP(ANum,i) = P0(ANum) + DeltaXa0(ANum) + Dir(ANum)*Vmax(ANum)*(DeltaT2(ANum))+ Dir(ANum)*Vmax(ANum)*(time-T2(ANum)) -0.5*Dir(ANum)*A(ANum)*(time-T2(ANum))^2;
                end
            end
         end
    end

end





