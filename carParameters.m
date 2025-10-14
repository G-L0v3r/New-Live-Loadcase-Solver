function p = carParameters(hPoints)

%Defining top level vehicle parameters
p.carMass = 327.6; % kg
p.wheelbase = 1735; % mm
p.trackWidth.F = 1150; % mm
p.trackWidth.R = 1100; % mm
p.cogHeight = 275.3; % mm
p.longDist.M = 0.554; % fraction rear
p.longDist.P = 0.525; % fraction rear
p.cL = 4.14;
p.cD = 1.51;
p.frontArea = 1.237; % m^2
p.airRho = 1.225; % kg m^-3
p.g = 9.81; % m s^-2
p.tyreRadius = hPoints.neutral.f.rhs.wheelCentre(3);

%Calulating derived parameters
p.weightTransfer.latF = (p.cogHeight * p.g * p.carMass * 0.5) / p.trackWidth.F; % N g^-1
p.weightTransfer.latR = (p.cogHeight * p.g * p.carMass * 0.5) / p.trackWidth.R; % N g^-1
p.weightTransfer.long = (p.cogHeight * p.g * p.carMass) / p.wheelbase; % N g^-1

end