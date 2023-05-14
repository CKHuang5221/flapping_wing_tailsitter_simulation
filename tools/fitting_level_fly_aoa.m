clc
clear all

rho = 1.196;
mg = 0.15*9.81;      %bi-x-wing
s = (330*140+340*100*2)/1000000; %(m^2) ,bi-x-wing
sweep_angle = 0;     %(degree)
AR = (72/23);           %aspect ratio
cl_alpha = (2*pi*cos(deg2rad(0))) / ( 2*cos(deg2rad(0))/AR + sqrt( 1 + (2*cos(deg2rad(0))/AR)^2 ) );
k0 = 0.87;           %Oswald's efficiency factor
cd0 = 0.02;          %drag coefficient from skin friction
index = 1;

for aoa = deg2rad(0.1):deg2rad(0.1):deg2rad(75)
Cl = cl_alpha*aoa;
Cd = cd0 + (Cl^2) / (pi*k0*AR);
V(index) = sqrt( (mg) / ( (1/2)*rho*s*(Cl + Cd*tan(aoa))) ) ;
AOA(index) = aoa;
index = index+1;
end

%then, use matlab curve fitting tool in command windows
cftool(V,AOA)
