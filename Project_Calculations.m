%This is the file for project calculations
%Andrew Seltz
clear
clc
close all





% Detailed Design calculations







%% Power Transmission and Conversion

%Define variables
gearloss = .03;



%% Gears and hand crank
%i. Specify: Number of teeth, module, material, face width, connection to
%shafts (torque transfer)
%ii. Select actual gears from a manufacturer
%iii. Analysis to confirm gears will meet functional requirements and not fail
%iv. Specify crank arm length and approximate width/thickness (detailed design
%not necessary)
%v. Crank arm basic yield analysis
%vi. Drawing of crank arm/gear assembly

Tensile_Force = 1000; % [N]
r_Drum = .1; % [m]

gearset1 = 4;
gearset2 = 4;

gear_ratio = gearset1 * gearset2;

T_in = Tensile_Force*r_Drum/gear_ratio

L_Crank = .15; % [m]

F_in = T_in/L_Crank %[N]

% Look at FBDs for force directions
%gear 2 Forces 
r2 = .05;
theta = 20*2*3.1415/360;

Ft32 = T_in/r2; % Tangential force on gear 2 from 3. 
Fr32 = Ft32*tan(theta); % Radial force

%gear 3 Forces
r3 = .2;
Ft23 = Ft32;
Fr23 = Ft32;
T3 = Ft23*r3*(1-gearloss);

%gear 4 Forces
r4 = .05;
T4 = T3;
Ft54 = T4/r4;
Fr54 = Ft54*tan(theta);

%gear 5 Forces
r5 = .2;
Fr45 = Fr54;
Ft45 = Ft54;
T5 = Ft45*r5*(1-gearloss);

%% Gear analysis code from HW
input_power = 60;  %[W]


N2 = 15;

N3 = 60;

N4 = 15;

N5 = 60;

Module = .002;

Sy = 310000000; %MPA
%above are input variables

speed_gear2 = input_power/T_in;
speed_gear3 = speed_gear2/4;
speed_gear4 = speed_gear3;
speed_gear5 = speed_gear4/4;


pitch_line_velocity = speed_gear5*r5;

Wt = Ft54;


Kv_cast = (3.05+pitch_line_velocity)/3.05;
Kv_milled = (6.1+pitch_line_velocity)/6.1;
Kv_hobbed = (3.56 + (pitch_line_velocity^.5))/3.56;
Kv_ground = ((5.56+(pitch_line_velocity)^.5)/5.56)^.5;






%solve for Facewidth

N = N5;
Y = 0.1365 + (1.273e-02)*N - (2.185e-04)*N^2 + (1.704e-6)*N^3 - (4.780e-9)*N^4;
Facewidth = Kv_milled*Wt/(Sy*Module*Y)















%% Support Shafts and Pullies


%% Bearings