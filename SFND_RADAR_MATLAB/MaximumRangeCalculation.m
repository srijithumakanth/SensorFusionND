clc
clear

%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;
PtdBm = 10*log10(Pt)+ 30;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8; % m/s

%TODO: Calculate the wavelength
lambda = c / fc; %in m

%TODO : Measure the Maximum Range a Radar can see. 
R = nthroot((PtdBm * G^2 * lambda^2 * RCS) / (Ps * (4*pi)^3), 4); 