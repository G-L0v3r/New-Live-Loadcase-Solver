function [tcpLoad] = tyreContactPatchVectorSetup(F, M)
%This  function creates a 6x1 vector of [Fx, Fy, Fz, Mx, My, Mz]^T acting
%on the tyre contact patch (TCP) for every tyre in every loading condition.

axle = fieldnames(F.x.static);
side = fieldnames(F.x.static.f);
loading = fieldnames(F.x);


for i = 1:length(loading)
    currentLoading = loading{i};
    for j = 1: length(axle)
        currentAxle = axle{j};
        for k = 1:length(side)
            currentSide = side{k};
                Fx = F.x.(currentLoading).(currentAxle).(currentSide);
                Fy = F.y.(currentLoading).(currentAxle).(currentSide);
                Fz = F.z.(currentLoading).(currentAxle).(currentSide);
                Mx = M.x.(currentLoading).(currentAxle).(currentSide);
                My = M.y.(currentLoading).(currentAxle).(currentSide);
                Mz = M.z.(currentLoading).(currentAxle).(currentSide);

                tcpLoad.(currentLoading).(currentAxle).(currentSide) = [Fx; Fy; Fz; Mx; My; Mz];
        end
    end
end
end
