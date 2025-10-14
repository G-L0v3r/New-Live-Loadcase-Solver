function [hPoints] = dynamicGeometryCreator(hPoints, F)
% This function creates an array for each loading condtion that represents
% whether each corner of the car is at full bump, full droop or neutrally
% positioned. It uses a user created matrix, dynamicSag, to decide how each
% corner at each loading is positioned. 
% 
% Possible improvements:
% A future upgrade would be add a
%function that compares Fz at each tyre duing each loading condition to a 
% reference full bump, full neutral or full sag value to decide how each
% corner is positioned, either quite simply by setting each corner to the
% closest geometry (i.e. Fz.brake.f.rhs is closer to Fzref.f.fullBump than 
% FzRef.f.neutral or FzRef.full droop, so the f rhs suspension is in full 
% bump during braking) or even more accurately by taking the %distance from 
% neutral to fully loaded and using rotation.m to estimate suspension 
% positions for each corner in each load.
% NG 10/1025

% dynamicSag are the numerical values of a 3d matrix in the format, with
% the rear values in the second page 'behind' the front values. e.g., 
% dynamicSag(3, 1, 2) is the accel, rhs, r corner dynamicSag value:
%           f.rhs  f.lhs  r.rhs  r.lhs
%static
%brake
%accel
%corner
%cornerAccel
%cornerBrake
%cone
%where 1 represents droop, 2 neutral and 3 bump.
dynamicSag.f = [2, 2,;
            3, 3;
            1, 1;
            1, 3;
            1, 3;
            1, 1;
            3, 3];
dynamicSag.r = [2, 2;
            2, 2;
            3, 3;
            1, 1;
            3, 3;
            3, 1;
            3, 3];
dynamicSag = cat(3, dynamicSag.f, dynamicSag.r);

loading = fieldnames(F.z);
axle = {'f', 'r'};
side = {'rhs', 'lhs'};
location = {'lwrOb', 'lwrFwdIb', 'lwrRwdIb', 'uprOb', 'uprFwdIb', 'uprRwdIb', 'prOb', 'prIb', 'trOb', 'trIb', 'wheelCentre'};
sag = {'droop', 'neutral', 'bump'};

for i = 1:length(loading)
    currentLoad = loading{i};
    for j = 1:length(axle)
        currentAxle = axle{j};
        for k = 1:length(side)
            currentSide = side{k};
            for l = 1:length(location)
                currentLocation = location{l};

                sagIndex = dynamicSag(i, k, j);
                currentSag = sag{sagIndex};
                currentHPointValue = hPoints.(currentSag).(currentAxle).(currentSide).(currentLocation);
                hPoints.dynamic.(currentLoad).(currentAxle).(currentSide).(currentLocation) = currentHPointValue;


            end
        end
    end
end


