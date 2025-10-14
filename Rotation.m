function [hPoints] = Rotation(hPoints)
%This function takes in the neutral RHS hardpoints imported by
%hardpointParameters, and creates two arrays of the RHS hardpoints, 
%one at full bump (max compression), one at full droop (max rebound). It
%does this by using 3 functions created by Vinnie; OBMove, Rotate, and
%ChangeZ.

%Rotate uses the Rodrigues matrix transformation. It creates an axis
%between the two upper hardpoints...
    totalZ = 50;
    zDroop = -totalZ/2;
    zBump = totalZ/2;
    numOfAxles = length(fieldnames(hPoints.neutral));
    axleNames = fieldnames(hPoints.neutral);
    hPoints.bump = hPoints.neutral;
    hPoints.droop = hPoints.neutral;


    for axleCounter = 1:numOfAxles
        axle = axleNames{axleCounter};
        [droopOBU.(axle), droopOBL.(axle), droopThetaU.(axle), droopThetaL.(axle), zz] = OBMove(hPoints, axle, zDroop);
        [bumpOBU.(axle), bumpOBL.(axle), bumpThetaU.(axle), bumpThetaL.(axle), zz] = OBMove(hPoints, axle, zBump);
        hPoints.droop.(axle).rhs.lwrOb = droopOBL.(axle);
        hPoints.droop.(axle).rhs.uprOb = droopOBU.(axle);
        uprNeutral2Droop = hPoints.droop.(axle).rhs.uprOb - hPoints.neutral.(axle).rhs.uprOb;
        lwrNeutral2Droop = hPoints.droop.(axle).rhs.lwrOb - hPoints.neutral.(axle).rhs.lwrOb;
        avNeutral2Droop = (uprNeutral2Droop + lwrNeutral2Droop)/2;
        hPoints.droop.(axle).rhs.wheelCentre = hPoints.neutral.(axle).rhs.wheelCentre + avNeutral2Droop;
        hPoints.droop.(axle).rhs.prOb = hPoints.neutral.(axle).rhs.prOb + avNeutral2Droop;
        hPoints.droop.(axle).rhs.trOb = hPoints.neutral.(axle).rhs.trOb + avNeutral2Droop;
        hPoints.bump.(axle).rhs.lwrOb = bumpOBL.(axle);
        hPoints.bump.(axle).rhs.uprOb = bumpOBU.(axle);
        uprNeutral2Bump = hPoints.bump.(axle).rhs.uprOb - hPoints.neutral.(axle).rhs.uprOb;
        lwrNeutral2Bump = hPoints.bump.(axle).rhs.lwrOb - hPoints.neutral.(axle).rhs.lwrOb;
        avNeutral2Bump = (uprNeutral2Bump + lwrNeutral2Bump)/2;
        hPoints.bump.(axle).rhs.wheelCentre = hPoints.neutral.(axle).rhs.wheelCentre + avNeutral2Bump;
        hPoints.bump.(axle).rhs.prOb = hPoints.neutral.(axle).rhs.prOb + avNeutral2Bump;
        hPoints.bump.(axle).rhs.trOb = hPoints.neutral.(axle).rhs.trOb + avNeutral2Bump;
    end

    function OB1 = Rotate(IBFS, IBRS, OB, theta) %Rotation using rodriguez matrix transformation. 
        theta = -theta;
        AxisVector = (IBFS - IBRS) / norm(IBFS - IBRS);
        K = [0,-AxisVector(3),AxisVector(2);AxisVector(3),0,-AxisVector(1);-AxisVector(2),AxisVector(1),0];
        R = [1,0,0;0,1,0;0,0,1] + sind(theta)*K + (1 - cosd(theta))*K^2;
        p1 = OB - IBRS;
        p2 = R*p1';
        OB1 = p2'+ IBRS;
    end
    
    function [OB1 , theta] = ChangeZ(IBFS, IBRS, OB, z) % binary search method to find angle to tranlsate and new OB location
        bumpOB = OB(3) + z;
        tOB = [0,0,0];
        tl = -10;
        tu = 10;
        while tOB(3) ~= bumpOB
            tt = (tl+tu)/2;
            tOB = Rotate(IBFS, IBRS, OB,tt);
            if tOB(3) > bumpOB
                tu = tt;
            end
            if tOB(3) < bumpOB
                tl =tt;
            end
        end
        theta = tt;
        OB1 = tOB;
    end
    
    function [OBU , OBL , thetaU, thetaL, zzz] = OBMove(hPoints, axle, z ) % matches lower wishbone location using binary search
        LIBFS = hPoints.neutral.(axle).rhs.lwrFwdIb;
        LIBRS = hPoints.neutral.(axle).rhs.lwrRwdIb;
        LOB = hPoints.neutral.(axle).rhs.lwrOb;
        UIBFS = hPoints.neutral.(axle).rhs.uprFwdIb;
        UIBRS = hPoints.neutral.(axle).rhs.uprRwdIb;
        UOB = hPoints.neutral.(axle).rhs.uprOb;
        OBDist = round(norm((UOB - LOB), 'fro'), 6);
        tUOB = [0,0,0];
        zl = z -2;
        zu = z +2;
        [tLOB,thetaLOB] = ChangeZ(LIBFS,LIBRS,LOB,z);
        while round(norm(tUOB - tLOB),6) ~= OBDist
            zz = (zl + zu)/2;
            [tUOB,thetaUOB] = ChangeZ(UIBFS,UIBRS,UOB,zz);
            if norm(tUOB - tLOB) > OBDist
                zu = zz;
            end
            if norm(tUOB - tLOB) < OBDist
                zl = zz;
            end
            
        end
        OBU = tUOB;
        OBL = tLOB;
        thetaU = thetaUOB;
        thetaL = thetaLOB;
        zzz = zz;
    end
end