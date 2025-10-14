function [memberForce, uprightForce, max, whenMax] = trussSolver(hPoints, p, tcpLoad, F)

axle = fieldnames(hPoints.neutral);
side = fieldnames(hPoints.neutral.f);
loading = fieldnames(F.x);
location = fieldnames(hPoints.neutral.f.rhs);
axes = fieldnames(F);
geometry = {'neutral', 'dynamic'};
member = {'wboneUprFwd', 'wboneUprRwd', 'wboneLwrFwd', 'wboneLwrRwd', 'pr', 'tr'};
uprightForce = struct();
maxForce = struct();

%creating the force and moment vectors.

%creating the shared neutral geo vector

for setupAxleCount = 1:length(axle)
    cAxle = axle{setupAxleCount};
    for setupSideCount = 1:length(side)
        cSide = side{setupSideCount};
        hPoints.neutral.(cAxle).(cSide).tcp = [hPoints.neutral.(cAxle).(cSide).wheelCentre(1), ...
            hPoints.neutral.(cAxle).(cSide).wheelCentre(2), hPoints.neutral.(cAxle).(cSide).wheelCentre(3) - p.tyreRadius];
        vector.neutral.(cAxle).(cSide).wboneUprFwd = hPoints.neutral.(cAxle).(cSide).uprOb - hPoints.neutral.(cAxle).(cSide).uprFwdIb;
        vector.neutral.(cAxle).(cSide).wboneUprRwd = hPoints.neutral.(cAxle).(cSide).uprOb - hPoints.neutral.(cAxle).(cSide).uprRwdIb;
        vector.neutral.(cAxle).(cSide).wboneLwrFwd = hPoints.neutral.(cAxle).(cSide).lwrOb - hPoints.neutral.(cAxle).(cSide).lwrFwdIb;
        vector.neutral.(cAxle).(cSide).wboneLwrRwd = hPoints.neutral.(cAxle).(cSide).lwrOb - hPoints.neutral.(cAxle).(cSide).lwrRwdIb;
        vector.neutral.(cAxle).(cSide).pr = hPoints.neutral.(cAxle).(cSide).prOb - hPoints.neutral.(cAxle).(cSide).prIb;
        vector.neutral.(cAxle).(cSide).tr = hPoints.neutral.(cAxle).(cSide).trOb - hPoints.neutral.(cAxle).(cSide).trIb;

        %setting up unit vectors for each suspension member

        vector.u.neutral.(cAxle).(cSide).wboneUprFwd = vector.neutral.(cAxle).(cSide).wboneUprFwd / norm(vector.neutral.(cAxle).(cSide).wboneUprFwd);
        vector.u.neutral.(cAxle).(cSide).wboneUprRwd = vector.neutral.(cAxle).(cSide).wboneUprRwd / norm(vector.neutral.(cAxle).(cSide).wboneUprRwd);
        vector.u.neutral.(cAxle).(cSide).wboneLwrFwd = vector.neutral.(cAxle).(cSide).wboneLwrFwd / norm(vector.neutral.(cAxle).(cSide).wboneLwrFwd);
        vector.u.neutral.(cAxle).(cSide).wboneLwrRwd = vector.neutral.(cAxle).(cSide).wboneLwrRwd / norm(vector.neutral.(cAxle).(cSide).wboneLwrRwd);
        vector.u.neutral.(cAxle).(cSide).pr = vector.neutral.(cAxle).(cSide).pr / norm (vector.neutral.(cAxle).(cSide).pr);
        vector.u.neutral.(cAxle).(cSide).tr = vector.neutral.(cAxle).(cSide).tr / norm(vector.neutral.(cAxle).(cSide).pr);

        %setting up the position vectors for each suspension member
        %relative to the tcp.

        vector.ibToTcp.neutral.(cAxle).(cSide).wboneUprFwd = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).uprFwdIb;
        vector.ibToTcp.neutral.(cAxle).(cSide).wboneUprRwd = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).uprRwdIb;
        vector.ibToTcp.neutral.(cAxle).(cSide).wboneLwrFwd = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).lwrFwdIb;
        vector.ibToTcp.neutral.(cAxle).(cSide).wboneLwrRwd = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).lwrRwdIb;
        vector.ibToTcp.neutral.(cAxle).(cSide).pr = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).prIb;
        vector.ibToTcp.neutral.(cAxle).(cSide).tr = hPoints.neutral.(cAxle).(cSide).tcp ...
            - hPoints.neutral.(cAxle).(cSide).trIb;

        %sets whether a push or pull rod is used (1 = pull, 2 = push, 
        % needed as a pull rod acts on the upper upright joint, and a push rod on the lower).
        if setupAxleCount == 1
            pushPullRodType.(cAxle) = 1;
        else
            pushPullRodType.(cAxle) = 2;
        end
        %This sets up the dynamic suspension geo in the example format 
        %'vector.dynamic.cornerBrake.f.Lhs.pr'
        for setupLoadingCount = 1:length(loading)
            cLoad = loading{setupLoadingCount};
            hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp = [hPoints.dynamic.(cLoad).(cAxle).(cSide).wheelCentre(1), ...
            hPoints.dynamic.(cLoad).(cAxle).(cSide).wheelCentre(2), hPoints.dynamic.(cLoad).(cAxle).(cSide).wheelCentre(3) - p.tyreRadius];

            vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprFwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).uprOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).uprFwdIb;
            vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprRwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).uprOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).uprRwdIb;
            vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrFwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrFwdIb;
            vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrRwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrRwdIb;
            vector.dynamic.(cLoad).(cAxle).(cSide).pr = hPoints.dynamic.(cLoad).(cAxle).(cSide).prOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).prIb;
            vector.dynamic.(cLoad).(cAxle).(cSide).tr = hPoints.dynamic.(cLoad).(cAxle).(cSide).trOb - hPoints.dynamic.(cLoad).(cAxle).(cSide).trIb;
    
            %setting up unit vectors for each suspension member
    
            vector.u.dynamic.(cLoad).(cAxle).(cSide).wboneUprFwd = vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprFwd / norm(vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprFwd);
            vector.u.dynamic.(cLoad).(cAxle).(cSide).wboneUprRwd = vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprRwd / norm(vector.dynamic.(cLoad).(cAxle).(cSide).wboneUprRwd);
            vector.u.dynamic.(cLoad).(cAxle).(cSide).wboneLwrFwd = vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrFwd / norm(vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrFwd);
            vector.u.dynamic.(cLoad).(cAxle).(cSide).wboneLwrRwd = vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrRwd / norm(vector.dynamic.(cLoad).(cAxle).(cSide).wboneLwrRwd);
            vector.u.dynamic.(cLoad).(cAxle).(cSide).pr = vector.dynamic.(cLoad).(cAxle).(cSide).pr / norm (vector.dynamic.(cLoad).(cAxle).(cSide).pr);
            vector.u.dynamic.(cLoad).(cAxle).(cSide).tr = vector.dynamic.(cLoad).(cAxle).(cSide).tr / norm(vector.dynamic.(cLoad).(cAxle).(cSide).pr);

            vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).wboneUprFwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).uprFwdIb;
        vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).wboneUprRwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).uprRwdIb;
        vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).wboneLwrFwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrFwdIb;
        vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).wboneLwrRwd = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).lwrRwdIb;
        vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).pr = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).prIb;
        vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).tr = hPoints.dynamic.(cLoad).(cAxle).(cSide).tcp ...
            - hPoints.dynamic.(cLoad).(cAxle).(cSide).trIb;

        end

    end
end

%Forming the first 3 rows of the force moment component matrices (fMMatrix). These 3
%rows represent the the x, y, and z contributions of each suspension member
%to reacting the TCP load. We will form the final 3 rows later, these
%represent the moment around the x, y, and z axes that each member
%contributes to resist the TCP load.
for forceLoadCount = 1:length(loading)
    cLoad = loading{forceLoadCount};
    for forceAxleCount = 1:length(axle)
        cAxle = axle{forceAxleCount};
        for forceSideCount = 1:length(side)
            cSide = side{forceSideCount};
            for forceMemberCount = 1:length(member)
                cMem = member{forceMemberCount};
                cDynamicV = transpose(vector.u.dynamic.(cLoad).(cAxle).(cSide).(cMem));
                %vector.u.dynamic.(cLoad).(cAxle).(cSide).wBoneUprFwd
                cNeutralV = transpose(vector.u.neutral.(cAxle).(cSide).(cMem));
                fMM.dynamic.(cLoad).(cAxle).(cSide)(1:3, (forceMemberCount)) = cDynamicV;
                fMM.neutral.(cLoad).(cAxle).(cSide)(1:3, (forceMemberCount)) = cNeutralV;
                max.tensile.(cAxle).(cMem) = 0;
                whenMax.tensile.(cAxle).(cMem) ='N/A';
                max.compressive.(cAxle).(cMem) = 0;
                whenMax.compressive.(cAxle).(cMem) = 'N/A';
            end
        end
    end
end

%Setting up rows 4 - 6 of the fMM, the contributions of each member to
%resisting the momemnts about the tcp. If we have a force vector F, and a 
%position vector from point O to any point along that force r, then the
%moment that F generates around O is equal in magnitude to, and around the
%axis of, the cross product vector between r and F, rxF. For our matrix, we
%are interested in the moment contribution of each suspension member around
%the tcp, so the moment vector is the cross product of a position vector
%(eg. f.rhs.uprFwdIb - f.rhs.tcp) and the force scalar * the unit vector or
%that member. This can be rearranged to the force scalar * (r x unit).
for momentLoadCount = 1:length(loading)
    cLoad = loading{momentLoadCount};
    for momentAxleCount = 1:length(axle)
        cAxle = axle{momentAxleCount};
        for momentSideCount = 1:length(side)
            cSide = side{momentSideCount};
            for momentMemberCount = 1:length(member)
                cMem = member{momentMemberCount};
                cDynamicIbtoTcp = transpose(vector.ibToTcp.dynamic.(cLoad).(cAxle).(cSide).(cMem));
                cNeutralIbtoTcp = transpose(vector.ibToTcp.neutral.(cAxle).(cSide).(cMem));
                cDynamicV = transpose(vector.u.dynamic.(cLoad).(cAxle).(cSide).(cMem));
                cNeutralV = transpose(vector.u.neutral.(cAxle).(cSide).(cMem));
                fMM.dynamic.(cLoad).(cAxle).(cSide)(4:6, (momentMemberCount)) = cross(cDynamicIbtoTcp, cDynamicV);
                fMM.neutral.(cLoad).(cAxle).(cSide)(4:6, (momentMemberCount)) = cross(cNeutralIbtoTcp, cNeutralV);
            end
        end
    end
end

%And now we can FINALLY solve our system of equations:
for solveLoadCount = 1:length(loading)
    cLoad = loading{solveLoadCount}; %sets the current loadcase event
    for solveGeoCount = 1:length(geometry)
       cGeo = geometry{solveGeoCount}; %sets whether the suspension is in it's neutral or dynamic position
        for solveAxleCount = 1:length(axle)
            cAxle = axle{solveAxleCount}; %sets F or R axle
            for solveSideCount = 1:length(side)
                cSide = side{solveSideCount}; %sets Rhs or Lhs
                cFMM = fMM.(cGeo).(cLoad).(cAxle).(cSide);
                cTcpLoad = tcpLoad.(cLoad).(cAxle).(cSide);
                r = mldivide(cFMM, cTcpLoad);
                
                for solveMemberCount = 1:length(member)
                    cMem = member{solveMemberCount};
                    R.(cLoad).(cGeo).(cAxle).(cSide).(cMem) = r(solveMemberCount);
                    if R.(cLoad).(cGeo).(cAxle).(cSide).(cMem) > max.tensile.(cAxle).(cMem)
                        max.tensile.(cAxle).(cMem) = R.(cLoad).(cGeo).(cAxle).(cSide).(cMem);
                        whenMax.tensile.(cAxle).(cMem) = strjoin({(cLoad),(cGeo),(cSide)}, {' loadcase, ', ' geo, '});
                    elseif R.(cLoad).(cGeo).(cAxle).(cSide).(cMem) < max.compressive.(cAxle).(cMem)
                        max.compressive.(cAxle).(cMem) = R.(cLoad).(cGeo).(cAxle).(cSide).(cMem);
                        whenMax.compressive.(cAxle).(cMem) = strjoin({(cLoad),(cGeo),(cSide)}, {' loadcase ', ' geo, '});
                    end
                end
            end
        end
    end
end

%And now we have out force magnitude for each suspension member, we can
%process this to make it a bit more readable.

for processLoadCount = 1:length(loading)
    cLoad = loading{processLoadCount};
    for processAxleCount = 1:length(axle)
        cAxle = axle{processAxleCount}; %sets F or R axle
        for processSideCount = 1:length(side)
            cSide = side{processSideCount};
            for processMemberCount = 1:length(member)
                cMem = member{processMemberCount};
                cRNeutral = R.(cLoad).neutral.(cAxle).(cSide).(cMem); %cR is the current resultant load
                cRDynamic = R.(cLoad).dynamic.(cAxle).(cSide).(cMem);
                memberForce.(cLoad).neutral.(cAxle).(cSide).(cMem) = cRNeutral * vector.u.neutral.(cAxle).(cSide).(cMem);
                memberForce.(cLoad).neutral.(cAxle).(cSide).(cMem)(4) = cRNeutral;
                memberForce.(cLoad).dynamic.(cAxle).(cSide).(cMem) = cRDynamic * vector.u.dynamic.(cLoad).(cAxle).(cSide).(cMem);
                memberForce.(cLoad).dynamic.(cAxle).(cSide).(cMem)(4) = cRDynamic;
            end
            for processGeoCount = 1:length(geometry)
                cTcpLoad = tcpLoad.(cLoad).(cAxle).(cSide);
                cGeo = geometry{processGeoCount};
                uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr = - (memberForce.(cLoad).(cGeo).(cAxle).(cSide).tr(:,1:3));
                cWboneUprFwd = (memberForce.(cLoad).(cGeo).(cAxle).(cSide).wboneUprFwd(1,1:3))';
                cWboneUprRwd = (memberForce.(cLoad).(cGeo).(cAxle).(cSide).wboneUprRwd(1,1:3))';
                cWboneLwrFwd = (memberForce.(cLoad).(cGeo).(cAxle).(cSide).wboneLwrFwd(1,1:3))';
                cWboneLwrRwd = (memberForce.(cLoad).(cGeo).(cAxle).(cSide).wboneLwrRwd(1,1:3))';
                if pushPullRodType.(cAxle) == 1
                    uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr = transpose(-(cWboneLwrFwd + cWboneLwrRwd));
                    uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr = transpose(-(cTcpLoad(1:3,1) + (uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr)' + (uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr)'));
                elseif pushPullRodType.(cAxle) == 2
                    uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr = transpose(-(cWboneUprFwd + cWboneUprRwd));
                    uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr = transpose(-(cTcpLoad(1:3,:) + (uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr)' + (uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr)'));
                end
                cUprX = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr(1);
                cUprY = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr(2);
                cUprZ = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr(3);
                uprightForce.(cLoad).(cGeo).(cAxle).(cSide).upr(4) = sqrt((cUprX)^2 + (cUprY)^2 + (cUprZ)^2);
                cLwrX = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr(1);
                cLwrY = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr(2);
                cLwrZ = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr(3);
                uprightForce.(cLoad).(cGeo).(cAxle).(cSide).lwr(4) = sqrt((cLwrX)^2 + (cLwrY)^2 + (cLwrZ)^2);
                cTrX = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr(1);
                cTrY = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr(2);
                cTrZ = uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr(3);
                uprightForce.(cLoad).(cGeo).(cAxle).(cSide).tr(4) = sqrt((cTrX)^2 + (cTrY)^2 + (cTrZ)^2);
            end

        end
    end
end



end




                









