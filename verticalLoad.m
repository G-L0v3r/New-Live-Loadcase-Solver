function [F, FzPercent] = verticalLoad(p, load)
 
    F.z = struct();
    FzSum = struct();
%{
    FzSum.static = 0;
    FzSum.brake = 0;
    FzSum.accel = 0;
    FzSum.corner = 0;
    FzSum.cornerAccel = 0;
    FzSum.cornerBrake = 0;
 %}
    FzPercent = struct();
    axle = {'f', 'r'};
    side = {'rhs', 'lhs'};
    loading = fieldnames(load);
    for loadCount = 1:length(loading)
        cLoad = loading{loadCount};
        FzSum.(cLoad) = 0;
    end

    for i = 1:length(loading)
        currentLoad = loading{i};
        for j = 1:length(axle)
            currentAxle = axle{j};
            for k = 1:length(side)
                currentSide = side{k};
                % set parameters based on F or R axle
                if j == 1 
                    longDistM = (1 - p.longDist.M);
                    longDistP = (1 - p.longDist.P);
                    latWT = p.weightTransfer.latF;
                    axleMultiplier = -1; % Load is removed from front during accel
                else
                    longDistM = p.longDist.M;
                    longDistP = p.longDist.P;
                    latWT = p.weightTransfer.latR;
                    axleMultiplier = 1; % Load is added to rear during accel
                end

                %left or right multiplier
                if k == 1
                    sideMultiplier = 1;
                else
                    sideMultiplier = -1;
                end
                
                %load components
                longitudinal_load = 0.5 * (p.carMass * p.g * longDistM);
                downforce_load = 0.5 * (load.(currentLoad).downforce * longDistP);
                long_accel_load = axleMultiplier * (0.5 * (p.weightTransfer.long * load.(currentLoad).longG));
                lat_accel_load = sideMultiplier * (latWT * load.(currentLoad).latG);
                vert_accel_load = 0.25 * (load.(currentLoad).vertSI * p.carMass);

                % Find resultant corner weight
                F.z.(currentLoad).(currentAxle).(currentSide) = longitudinal_load + downforce_load + long_accel_load + lat_accel_load + vert_accel_load;
                FzCurrent = F.z.(currentLoad).(currentAxle).(currentSide);
                FzSum.(currentLoad) = FzSum.(currentLoad) + FzCurrent;

            end
        end


       
    end

        for a = 1:length(loading)
        currentLoad = loading{a};
            for b = 1:length(axle)
                currentAxle = axle{b};
                for c = 1:length(side)
                    currentSide = side{c};
                    FzPercent.(currentLoad).(currentAxle).(currentSide) = F.z.(currentLoad).(currentAxle).(currentSide) / FzSum.(currentLoad);
                end
            end
        end


end