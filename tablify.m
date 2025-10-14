function [allWishboneLoads, maxWishboneLoads, allUprightLoads, maxWishboneLoadsWhen] = tablify(memberForce, uprightForce, max, whenMax)

tensComp = fieldnames(max);
axle = fieldnames(max.tensile);
member = fieldnames(max.tensile.f);
loading = fieldnames(memberForce);
geometry = fieldnames(memberForce.accel);
side = fieldnames(memberForce.accel.dynamic.f);
uprightBearingLocation = fieldnames(uprightForce.static.neutral.f.rhs);
allWishboneLoads = [];
maxWishboneLoads = [];


for i = 1:length(loading)
    cLoad = loading{i}; %sets the current loadcase event
    for j = 1:length(geometry)
       cGeo = geometry{j}; %sets whether the suspension is in it's neutral or dynamic position
       tempWboneAxleTable = zeros(24,4);
       tempUprightAxleTable = zeros(12,4);
        for k = 1:length(axle)
            cAxle = axle{k}; %sets F or R axle
            tempWboneSideTable = zeros(12,4);
            tempUprightSideTable = zeros(6,4);
            for l = 1:length(side)
                cSide = side{l}; %sets Rhs or Lhs
                tempWboneMemTable = zeros(6, 4); %set up a temporary table to store the current loads)
                tempUprightBearingTable = zeros(3,4);
                for m = 1:length(member)
                    cMem = member{m};
                    tempWboneMemTable(m,1:4) = [memberForce.(cLoad).(cGeo).(cAxle).(cSide).(cMem)];
                end
                for n = 1:length(uprightBearingLocation)
                    cUBL = uprightBearingLocation{n};
                    tempUprightBearingTable(n,1:4) = [uprightForce.(cLoad).(cGeo).(cAxle).(cSide).(cUBL)];
                end
                tempUprightSideTable(((3 * l) - 2):(3*l),(1:4)) = tempUprightBearingTable;
                tempWboneSideTable(((6 * l) - 5):(6 * l), 1:4) = tempWboneMemTable;
            end
            tempUprightAxleTable(((6 * k) - 5):6 * k, 1:4) = tempUprightSideTable;
            tempWboneAxleTable(((12 * k) - 11):12 * k, 1:4) = tempWboneSideTable;
            
        end
        tempTable.(cLoad).(cGeo) = tempWboneAxleTable;
        tempUprightTable.(cLoad).(cGeo) = tempUprightAxleTable;
    end
end

i = 1;
for i = 1:length(loading)
    cLoad = loading{i}; %sets the current loadcase event
       allWishboneLoads(1:24, (8*i)-7:8*i) = [tempTable.(cLoad).dynamic tempTable.(cLoad).neutral];
       allUprightLoads(1:12, (8*i)-7:8*i) = [tempUprightTable.(cLoad).dynamic tempUprightTable.(cLoad).neutral];
end

i = 1;
for i = 1:length(member)
    cMem = member{i};
    maxWishboneLoads(i,1) = [max.tensile.f.(cMem)];
    maxWishboneLoads(i+6,1) = [max.compressive.f.(cMem)];
    maxWishboneLoads(i+12,1) = [max.tensile.r.(cMem)];
    maxWishboneLoads(i+18,1) = [max.compressive.r.(cMem)];

    maxWishboneLoadsWhen(i,1) = {whenMax.tensile.f.(cMem)};
    maxWishboneLoadsWhen(i+6,1) = {whenMax.compressive.f.(cMem)};
    maxWishboneLoadsWhen(i+12,1) = {whenMax.tensile.r.(cMem)};
    maxWishboneLoadsWhen(i+18,1) = {whenMax.compressive.r.(cMem)};
end





end





                  









