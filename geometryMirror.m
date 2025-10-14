function [hPoints] = geometryMirror(hPoints)

sag = {'droop', 'bump', 'neutral'};

axle = {'f', 'r'};
%side = {'rhs','lhs'};
location = {'lwrOb', 'lwrFwdIb', 'lwrRwdIb', 'uprOb', 'uprFwdIb', 'uprRwdIb', 'prOb', 'prIb', 'trOb', 'trIb', 'wheelCentre'};
numSag = length(sag);
numAxles = length(axle);
%numSides = length(side);
numLocations = length(location);

for i = 1:numSag
    currentSag = sag{i};
    for j = 1:numAxles
        currentAxle = axle{j};
        for k = 1:numLocations
            currentLocation = location{k};
            currentXValue = hPoints.(currentSag).(currentAxle).rhs.(currentLocation)(1); 
            currentRhsYValue = hPoints.(currentSag).(currentAxle).rhs.(currentLocation)(2);
            currentZValue = hPoints.(currentSag).(currentAxle).rhs.(currentLocation)(3);

            currentLhsYValue = currentRhsYValue * -1;

            hPoints.(currentSag).(currentAxle).lhs.(currentLocation) = [currentXValue, currentLhsYValue, currentZValue];


        end
    end
end

            


