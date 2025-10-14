function hPoints = hardpointParameters()
%Define hardpoints at 50% travel ('neutral'). They can either come from a
%.csv, or they can be defined manually below. 
% 
%Importing a .csv: 
%conventional coordianate axes (x = long, y = lat, z = height), inputType = 1,
%importing NX coordinates:
%(x = lat, y = height, z = long), inputType = 2;
%
%Manual coordiate entry:
%inputType = 3.
    %{
    Hardpoints are in the format
    Front RHS LWR OB
    Front RHS LWR FWD IB
    Front RHS LWR RWD IB
    Front RHS UPR OB
    Front RHS UPR FWD IB
    Front RHS UPR RWD IB
    Front RHS PR OB
    Front RHS PR IB
    Front RHS TR OB
    Front RHS TR IB
    Front RHS Wheel Centre
    Rear RHS LWR OB
    Rear RHS LWR FWD IB
    Rear RHS LWR RWD IB
    Rear RHS UPR OB
    Rear RHS UPR FWD IB
    Rear RHS UPR RWD IB
    Rear RHS PR OB
    Rear RHS PR IB
    Rear RHS TR OB
    Rear RHS TR IB
    Rear RHS Wheel Centre
    %}

    inputType = 1;
    
    if inputType == 1
        hardpointsTable = readtable('hardpoints.csv');
        hardpoints = table2array(hardpointsTable);
    elseif inputType == 2
        tempTable = readtable('hardpoints.csv');
        tempArray = table2array(tempTable);
        hardpoints(:,1) = tempArray(:,3);
        hardpoints(:,2) = tempArray(:,1);
        hardpoints(:,3) = tempArray(:,2);


    else
        hardpoints = [] ;
    end

    axle = {'f', 'r'};
    side = {'rhs'};
    location = {'lwrOb', 'lwrFwdIb', 'lwrRwdIb', 'uprOb', 'uprFwdIb', 'uprRwdIb', 'prOb', 'prIb', 'trOb', 'trIb', 'wheelCentre'};
    numAxles = length(axle);
    numSides = length(side);
    numLocations = length(location);

    for i = 1:numAxles
        currentAxle = axle{i};
        for j = 1:numSides
            currentSide = side{j};
            for k = 1:numLocations
                currentLocation = location{k};
                if i == 1
                    index = k;
                else
                    index = k + numLocations;
                end
                hPoints.neutral.(currentAxle).(currentSide).(currentLocation) = hardpoints(index,:);

            end
        end
    end
%{   
    neutralHardpoints.f.rhs.lwrOb = hardpoints(1,:);
    neutralHardpoints.f.rhs.lwrFwdIb = hardpoints(2,:);
    neutralHardpoints.f.rhs.lwrRwdIb = hardpoints(3,:);
    neutralHardpoints.f.rhs.uprOb = hardpoints(4,:);
    neutralHardpoints.f.rhs.uprFwdIb = hardpoints(5,:);
    neutralHardpoints.f.rhs.uprRwdIb = hardpoints(6,:);
    neutralHardpoints.f.rhs.prOb = hardpoints(7,:);
    neutralHardpoints.f.rhs.prIb = hardpoints(8,:);
    neutralHardpoints.f.rhs.trOb = hardpoints(9,:);
    neutralHardpoints.f.rhs.trIb = hardpoints(10,:);
    neutralHardpoints.f.rhs.wheelCentre = hardpoints(11,:);
    
    neutralHardpoints.r.rhs.lwr.ob = hardpoints(12,:);
    neutralHardpoints.r.rhs.lwr.fwdIb = hardpoints(13,:);
    neutralHardpoints.r.rhs.lwr.rwdIb = hardpoints(14,:);
    neutralHardpoints.r.rhs.upr.ob = hardpoints(15,:);
    neutralHardpoints.r.rhs.upr.fwdIb = hardpoints(16,:);
    neutralHardpoints.r.rhs.upr.rwdIb = hardpoints(17,:);
    neutralHardpoints.r.rhs.pr.ob = hardpoints(18,:);
    neutralHardpoints.r.rhs.pr.ib = hardpoints(19,:);
    neutralHardpoints.r.rhs.tr.ob = hardpoints(20,:);
    neutralHardpoints.r.rhs.tr.ib = hardpoints(21,:);
    neutralHardpoints.r.rhs.wheelCentre = hardpoints(22,:);
%}
end







