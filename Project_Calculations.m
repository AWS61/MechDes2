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

pullstress = pulleyshaft(Tensile_Force);
[maxstress3, maxstress4] = gearshaft(Ft54, Fr54, Ft23, Fr23, T3, T_in, F_in, Tensile_Force, r_Drum);

function [maxstress3, maxstress4] = gearshaft(ft54, fr54, ft23, fr23, T3, T_in, F_in, Tensile_force, r_Drum)
    
    % shaft lengths and 
    l23 = 51*10^-3;   
    l34 = 40*10^-3;
    l45 = 41*10^-3;
    lcrank = 5*10^-3;
    d4 = 15*10^-3;
    d3 = 15*10^-3;
    l = l23 + l34 + l45; % total length
    
    % Reaction forces on gear shafts 2
    Roy = (ft54*(l23+l34)-ft23*l23)/(l);   
    Rby = ft23+Roy-ft54;
    Roz = (fr54*(l23+l34)-fr23*l23)/(l);
    Rbz = fr23+Roz-fr54;
    
    %Reaction forces on gear shaft 1
    R2oy = (ft23*l23-F_in*lcrank)/(l);
    R2by = R2oy - ft54 + Tensile_force;
    R2bz = fr23*(l45+l34)/l;
    R2oz = fr23-R2bz;
    % moments for gear shaft 1
    My2 = R2oy*(l23+l34);
    Mz2 = R2oz*(l23+l34);

    % moments for shaft 2
    My4 = Roy*l45;  % or  whichever is bigger
    My3 = Rby*l23;
    Mz4 = Roz*l45;  % or 
    Mz3 = Rbz*l23;

    % forces on gear shaft 3
    R3by = (-ft54*l45+Tensile_force*(l34+l45))/l;
    R3oy = -ft54-R3by+Tensile_force
    R3bz = (fr54*l45)/l;
    R3oz = fr54-R3bz
    T_out = Tensile_force*r_Drum;

    %moments in shaft 3
    My5 = R3oy*l45;
    Mz5 = R3oz*l45;
    MyD = R3by*l23;
    MzD = R3bz*l23;


    % max alternating at each gear or drum
    M2 = sqrt(My2^2+Mz2^2)/2;
    M3 = sqrt(My3^2+Mz3^2)/2;
    M4 = sqrt(My4^2+Mz4^2)/2;
    M5 = sqrt(My5^2+Mz5^2)/2;
    MD = (596*(l45)+(596-515)*(l23+l34))/2;   % hand done new calculation for max moment with updated minimum shaft diamater

   
    
    % prelinary stress concentrations for 
    gearKt = 1.4;
    gearKts = 1.25;

    maxstress2 = (sqrt((32*gearKt*M2/(pi*d4^3))^2+3*(16*gearKts*T_in/(pi*d4^3))))*10^-6;
    maxstress3 = (sqrt((32*gearKt*M3/(pi*d3^3))^2+3*(16*gearKts*T3/(pi*d3^3))))*10^-6;
    maxstress4 = (sqrt((32*gearKt*M4/(pi*d4^3))^2+3*(16*gearKts*T3/(pi*d4^3))))*10^-6;
    maxstress5 = (sqrt((32*gearKt*M5/(pi*d4^3))^2+3*(16*gearKts*T_out/(pi*d4^3))))*10^-6;
    
    % solving for minimum shaft diamater
    syms d
    assume(d >= 0)
    maxstressD = (sqrt((32*gearKt*MD/(pi*d^3))^2+3*(16*gearKts*T_out/(pi*d^3))))*10^-6 == 310/2;
    Min_d = solve(maxstressD, d, Real=true);
    dmin = double(Min_d);

    disp(['The minimum shaft diameter is ', num2str(dmin*10^3), ' [mm]'])
    
    disp(['von Mises stress at gear 2 = ', string(maxstress2), '[MPa]'])
    disp(['von Mises stress at gear 3 = ', string(maxstress3), '[MPa]'])
    disp(['von Mises stress at gear 4 = ', string(maxstress4), '[MPa]'])
    disp(['von Mises stress at gear 5 = ', string(maxstress5), '[MPa]'])
    disp(['with a shaft length of ', num2str(l)])
    
end

    %% pulley shaft analysis

function [Min_pul_d] = pulleyshaft(t);
      %pulley tension in N
    pulKt = 1.0; % pulley stess concentration
    pullen = 1; % length of pulley shaft in m
    %puld = .0254*1.4; % pulley shaft diameter
    
    % maximum pulley stress
    syms puld
    Pulley_stress = 310/2 == (32*pulKt*t*(pullen/2)/(pi*puld^3))*10^-6;

    Min_d = solve(Pulley_stress, puld, Real=true);
    Min_pul_d = double(Min_d);
    
    disp(["minimum pulley shaft diameter is ",num2str(Min_pul_d*39.37), " [in]"])
end



%% Bearings



