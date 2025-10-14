function load = loadParameters(p)

%Setting out the names for each loadcase and the variables they contain
loadcaseNames = {'static', 'brake','accel','corner','cornerAccel','cornerBrake', 'cone'};
loadcaseVariableNames = {'ve','latG','longG','vertG','latSI', 'longSI', 'vertSI','downforce','drag'};

%Input the speed (ms^-2), lateral acceleration (g), longitudinal
%acceleration (g), and vertical acceleration (g) for each load case.
loadcaseInput.static = [0, 0, 0, 0];
loadcaseInput.brake = [28.5, 0, -2.72, 0,]; %braking inputs: speed, lat G, long G, vert G
loadcaseInput.accel = [16.7, 0, 1.35, 0,]; %accel inputs: speed, lat G, long G, vert G
loadcaseInput.corner = [28.5, -2.31, 0, 0,]; %cornering inputs: speed, lat G, long G, vert G
loadcaseInput.cornerAccel = [28.5, -1.63, 0.92, 0,]; %cornerAccel mix inputs: speed, lat G, long G, vert G
loadcaseInput.cornerBrake = [28.5, -1.63, -1.92, 0,]; %cornerBrake mix speedinputs: , lat G, long G, vert G
loadcaseInput.cone = [28.5, 0, 0, 3,]; %cornerBrake mix speedinputs: , lat G, long G, vert G

numNames = length(loadcaseNames);
numVariables = length(loadcaseVariableNames);

load = struct();

%Completing the input arrays by calcuating accelerations in SI units, and
%aero forces
for i = 1:numNames
    currentLoadcaseName = loadcaseNames{i};
    for j = 1:numVariables
        currentLoadcaseVariable = loadcaseVariableNames{j};
        if j < 5
        currentLoadcaseValue = loadcaseInput.(currentLoadcaseName)(j);
        load.(currentLoadcaseName).(currentLoadcaseVariable) = currentLoadcaseValue;
        elseif j < 8
        load.(currentLoadcaseName).(currentLoadcaseVariable) = loadcaseInput.(currentLoadcaseName)(j-3) * p.g;
        elseif j == 8
        load.(currentLoadcaseName).(currentLoadcaseVariable) = 0.5 * p.cL * p.frontArea * p.airRho * loadcaseInput.(currentLoadcaseName)(1)^2;
        elseif j == 9
        load.(currentLoadcaseName).(currentLoadcaseVariable) = 0.5 * p.cD * p.frontArea * p.airRho * loadcaseInput.(currentLoadcaseName)(1)^2;
        end
             

    end
end

end