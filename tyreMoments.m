function [M] = tyreMoments(F)
M.x = struct(); %overturning loads
M.y = struct(); %rotaion loads
M.z = struct(); %aligning loads
axle = {'f', 'r'};
side = {'rhs', 'lhs'};
loading = fieldnames(F.z);



    for i = 1:length(loading)
        currentLoad = loading{i};
        for j = 1:length(axle)
            currentAxle = axle{j};
            for k = 1:length(side)
                currentSide = side{k};
                Fx = F.x.(currentLoad).(currentAxle).(currentSide);
                Fy = F.y.(currentLoad).(currentAxle).(currentSide);
                Fz = F.z.(currentLoad).(currentAxle).(currentSide);

                if i == 5
                    momentScale = ( Fy  /( -Fx + Fy ));
                elseif i == 6
                    momentScale = ( Fy  /( Fx + Fy ));
                else
                    momentScale = 1;
                end



                if Fx == 0 && Fy == 0
                    M.x.(currentLoad).(currentAxle).(currentSide) = 0;
                    M.y.(currentLoad).(currentAxle).(currentSide) = 0;
                    M.z.(currentLoad).(currentAxle).(currentSide) = 0;
                else
                    M.x.(currentLoad).(currentAxle).(currentSide) = -0.65 * ((0.0512 * -Fz )+8.18) * momentScale * 10^3;
                    M.y.(currentLoad).(currentAxle).(currentSide) = 0;
                    M.z.(currentLoad).(currentAxle).(currentSide) = -0.65 * ((0.0227 *-Fz)+ 7.2517) * momentScale * 10^3;                    
                end

            end
        end
    end
end
