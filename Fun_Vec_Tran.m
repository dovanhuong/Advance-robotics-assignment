function [ResultX ResultY ResultZ]=Fun_Vec_Tran(InputX,InputY,InputZ,TransMatrix)

    [Check_M,Check_K]=size(InputX);
    
    for Checki=1:Check_K
        PostionEnd=[InputX(Checki);InputY(Checki);InputZ(Checki);1];
        NewPosition = TransMatrix * PostionEnd;
             
        if Checki == 1
            ResultX = NewPosition(1);
            ResultY = NewPosition(2);
            ResultZ = NewPosition(3);
        else
            ResultX = [ResultX,NewPosition(1)];
            ResultY = [ResultY,NewPosition(2)];
            ResultZ = [ResultZ,NewPosition(3)];
        end
            
    end
    
end