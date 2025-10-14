function [F] = longAndLatLoad(F, FzPercent, p, load)

%F.x = struct();
%F.y = struct();
axle = {'f', 'r'};
side = {'rhs', 'lhs'};
loading = fieldnames(load);
%mBrake = [426, 180, 283, 120, 0];



    for i = 1:length(loading)
        currentLoad = loading{i};
        for j = 1:length(axle)
            currentAxle = axle{j};
            for k = 1:length(side)
                currentSide = side{k};
                if i == 1 %when i is one, static load so no drag or longitudinal g contribution to forces
                    dragMultiplier = 0;
                    longGContribution = 0;
                    b = 1; %b is the brake torque multiplier. When braking on the front
                           %axle, the brake torque is reacted by the
                           %wishbone and trackrid attachment points. To
                           %account for this, the force at the contact
                           %patch to decelerate the car (except the drag)
                           %is doubled by multiplying the longitudinal load
                           %from the acceleration (longAccelLoad) by 2, as
                           %the brake mount reaction torque around the wheel 
                           %centre generates the same force at the contact
                           %patch as longAccelLoad.
                           
                elseif i == 2
                    dragMultiplier = -0.25;
                    longGContribution = -load.(currentLoad).longSI* FzPercent.(currentLoad).(currentAxle).(currentSide);
                    if j == 1  
                        b = 2;
                    elseif j == 2
                        b = 1;
                    end
                elseif i == 3 || i == 4
                    b = 1;
                    if j == 1
                        dragMultiplier = 0;
                        longGContribution = 0;
                    else 
                        dragMultiplier = -0.5;
                        longGContribution = -load.(currentLoad).longSI*0.5;
                    end

                elseif i == 5
                    b = 1;
                     if j == 1
                        dragMultiplier = 0;
                        longGContribution = 0;
                    else 
                        dragMultiplier = -0.5;
                        longGContribution = -load.(currentLoad).longSI*0.5 * FzPercent.cornerAccel.(currentAxle).(currentSide)/FzPercent.accel.(currentAxle).(currentSide);
                     end
                elseif i == 6
                    dragMultiplier = -0.25;
                    longGContribution = -load.(currentLoad).longSI* FzPercent.(currentLoad).(currentAxle).(currentSide);
                    if j == 1
                        b = 2;
                    elseif j == 2
                        b = 1;
                    end

                end

                latAccelLoad = p.carMass * -load.(currentLoad).latSI * FzPercent.(currentLoad).(currentAxle).(currentSide);
                longAccelLoad = p.carMass * longGContribution * b;
                aeroLoad = load.(currentLoad).drag * dragMultiplier;
                F.x.(currentLoad).(currentAxle).(currentSide) = longAccelLoad + aeroLoad;
                F.y.(currentLoad).(currentAxle).(currentSide) = latAccelLoad;
            end
        end
    end
    end
