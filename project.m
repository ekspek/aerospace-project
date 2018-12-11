

clc
clear

%% Feasibility Design of a VTOL Aircraft
% Allows to test different VTOL configurations, disk loadings, propulsive 
% systems, number of rotors, rotors' type, batteries, among others.

%% Inputs
% General Inputs
g = 9.80665;            % Gravity acceleration (m/s2)
Pmax = 10;               % Maximum installed power (kW)
PF = 1.5;               % Installed Power Factor (PF) (-) - 1 for minimal hover capability; 2.7 id a typical for Turboshaft engines (Ref. Ng and Datta, 2018)
T_W = 1.3;              % Thrust-to-weight ratio [1.2 - 1.5], Ref. Raymer
rho = 1.225;            % Air Density (kg/m3)

% Mission related inputs
Endurance = 5;          % Endurance (h)
Reserve = 10/60;        % Reserve time (h) - 20 minutes
HoverTime = 180/3600;   % Hover time (h) - 90s + 90s (Take-off and Landing)
Vc = 30;                % Cruise Speed (m/s)

% Batteries related inputs (Assumptions)
Ebat = 165;     % energy specific density (W.h/kg) - Li-Ion batteries (Current State of the Art value)
Ebat_r = 0.1;   % batteries reserve (-) - 20% of reserve

% Electric Motors related inputs (Assumptions)
PWe = 4.5;      % Power to weight ratio (kW/kg) 
% Combustion Engine related inputs (Assumptions)
SFC = 0.600;
%SFC = 0.4*0.45359237;	% Specific Fuel Consumption (kg/(hp.hr)) [0.4-0.5] lb/(hp*hr)
PWg = 2.38;              % Power to weight ratio (kW/kg)

% Propeller Blade related items (Assumptions)
s = 0.1;        % propeller blade solidity (-)
cd0 = 0.01;     % base drag (-)
ki = 1.1;       % induced power coefficient, typical values [1.10 -1.15]
Mtip = 0.6;     % Mach number at the tip of the propeller blade (Mtip < 0.6 because of noise)
a = 340.294;    % Speed of sound (m/s) - Sea Level Conditions
Vtip = Mtip*a;  % Speed at the blade's tip (m/s)

% Weight related inputs (Assumptions based on Ref. Ng)
fwo = 0.15 ;  % assumption of 10% of the empty weight (We) is for avionics, electronics, interior systems, controllers
fws = 0.24; % structural weight fraction of the Take-Off Gross Weights

% Configuration Related Inputs (Multi-rotor)

%L_D = 1.5;              % lift-to-drag ratio (-) - if semi-empirical formula exists for the configuration introduce it in Step 3
DL = 35;	% disk loading (kg/m2) - disk loading can also be changed in the Iterative Process
rotor = 'unducted';     % ducted or unducted - the unducted propeller requires a higher hover power but causes less drag in cruise

np=0.4                  %efficiency - pc/np is the istalled power
%% Equations and consideration based on the following paper:
% W. Ng and A. Datta, "Development of Models for Electrochemical Power and
% Sizing of Electric-VTOL Aircraft", AIAA SciTech Forum, 2018.
% Also the classic helicopter textbook was used:
% W. Johnson, "Rotorcraft Aeromechnics", Cambridge University Press, 2013
% E. Torenbeek, "Advance Aircraft Design", John Wiley and Sons, 2013

%% Test Different number of rotors (NR)
for j = 1:41
    NR = 1 + (40-1)*(j-1)/40;
    
    error = 1;
    FoM(j) = 0.6;	% Initial guess for the Figure of Merit (FoM)
    FoM0 = FoM(j);
    while (error > 0.001)
        
        %% Step 1 - First estimate of the Gross Take-Off Weight (Wgto)
        FoM(j) = FoM0;	% (-) Initial guess for the Figure of Merit (FoM)
        if strcmpi(rotor,'ducted')
            Ph(j) = Pmax/PF/sqrt(2);	% (kW) Power @ Hover
        else
            Ph(j) = Pmax/PF;            % (kW) Power @ Hover
        end
        Wgto(j) = Ph(j)*1000*FoM(j)/(T_W*sqrt(DL*g/(2*rho)));   % (N)

        %% Step 2 - First estimate of the rotor radius and new estimation of FoM
        Th = T_W*Wgto(j);
        r(j) = sqrt(Th/(g*pi*NR*DL));	% (m)
        % Blade Mean lift coefficient - Hover Performance (Johnson, 2013)
        Ct = DL*g/(rho*Vtip*Vtip);      % (-)
        FoM(j) = 1/(ki+(3/4)*cd0/(6*ki*sqrt(Ct/2)*Ct/s)); % (-)
        
        %% Step 3 - Power required for cruise
        % Power @ cruise
        cd0c=0.03;                      %conditions for cruise - 0.01 - 0.02, assumed 0.03 due to high drag
        AR=10;
        k=1/(pi*AR*0.8);
        L_D = 0.94*(1/(2*sqrt(cd0c*k))); %empirical formula for the L/D
        Pc(j) = (Wgto(j)*Vc/L_D)/1000;     %(kW)
        
        %% Step 4 - Estimate structural weight from statistical databases
        Ws(j) = fws*Wgto(j); % (N)
        
        %% Step 5 - Estimate engine/motor weight based on power installed
        % For a Turboshaft propulsive system (including rotors)
        %HPh = Pmax*1.34102209;             % (hp)
        %Wp(j) = 1.8*HPh^(.9)*0.45359237*g;	% (N)
        
        % For a Turbogenerator sized for maximum power
        %Wr = 0;                             % (N), Rotors weight
        %Wpe(j) = Pmax*g/PWe;                % (N), Electric motors
        %Wpg(j) = Pmax*g/PWg;                % (N), Turbogenerator
        %Wp(j) = Wr + Wpe(j) + Wpg(j);       % (N), Propulsive System
        
        % For a Turbogenerator sized for cruise power   
        Wr = (5*0.050)*g;                   % (N), Rotors weight  ainda ter� de ser calculado
        Wpe(j) = Pmax*g/PWe;                % (N), Electric motors
        Wpg(j) = Pc(j)/np*g/PWg;            % (N), Turbogenerator sized for the installed power
        Wp(j) = Wr + Wpe(j) + Wpg(j);       % (N), Propulsive System
        
        %% Step 6 - Estimate fuel spent based on the endurance and specfic fuel consumption
        % Fully combustion system
        %Wfuel(j) = SFC*(Pc(j)*1.34102209*(Endurance+Reserve) + Ph(j)*1.34102209*HoverTime);  %(kg)
        %Wbat(j) = 0; %(kg)
        
        % Hybrid-electric system
        HPc = Pc(j)/np*1.341;                             %INSTALLED POWER Pc/np
       
        Wfuel(j) = SFC*(HPc*(Endurance+Reserve));          %(kg)
       
        %Wfuel(j)= Wgto(j)-(Wgto(j)/(exp(Endurance/((np/SFC)*(L_D)*(1/Vc))))) %o que mudei 
        %Wfuel(j)= Wgto(j)-(Wgto(j)/(exp((Endurance*9.81*SFC)/(L_D)))) %Breguet range equation
        
        
        Wbat(j) = Ph(j)*1000*HoverTime/(Ebat*(1-Ebat_r));	%(kg)
        
        error = abs((FoM(j) - FoM0)/FoM(j));
        FoM0 = FoM(j);
    end
    
    %% Step 7 - Estimate the empty weight
    We(j) = (Wp(j) + Ws(j))/(1-fwo); % (N)  peso eletr�nicos
    
    %% Step 8 - Payload
    Wuse(j) = Wgto(j) - We(j);                  % (N)
    Wpay(j) = Wuse(j) - Wfuel(j)*g - Wbat(j)*g; % (N)
end

%Design point calculation-------------------------------------------------------------------------------------------------------------------------
syms x;

%input data
rho=1.225;     
vstall=25;          %m/s
clmax=1.2;          %assumido, coef sustenta��o m�ximo   
cd0=0.03;
AR=10;
k=1/(pi*AR*0.8);    %0.8 nao deve ser

hold on;

% istalled power is different from the cruise power - this is the power
% required for the engine to cruise

Pinst=Pc(NR)/np;

%cruise speed -----------------------------------------------------------------------------------------------------------------------------
v=30;                 
cr=1/np*((rho*v^3*cd0)/(2*x)+((2*k)/(rho*v)*x));
ezplot(cr, [0 1000 0 15]);

hold on;

%climb angle - assumed 10 degrees----------------------------------------------------------------------------------------------------------------------------
v=sqrt(2/rho*x*sqrt(k/(cd0)));
clb=v/np*(sind(3)+((rho*v*v*cd0)/(2*x))+((2*k)/(rho*v*v))*x);
ezplot(clb, [0 1000 0 15]);

hold on;

%range------------------------------------------------------------------------------------------------------------------------------- 
v=30;                           %assumido m/s
ws=1/2*rho*v^2*sqrt(cd0/k);
x=[ws ws];
y=[0 15];
line(x,y);

hold on;

%endurance
v=30;
ws=1/2*rho*v^2*sqrt(3*cd0/k);
x=[ws ws];
y=[0 15];
line(x,y);

hold on;

%vstall criteria ----- Correct
ws=1/2*rho*vstall^2*clmax;
x=[ws ws];
y=[0 15];
line(x,y);

hold on; 

%Pot�ncia de cruseiro / Mtow
NR = 4                     %Number of rotors
Power_cruise_w = Pc(NR)*10^3/ Wgto(NR);
x=[0 1000];
y=[Power_cruise_w Power_cruise_w];
line(x,y);

%Installed power / MTOW - the one to use
x=[0 1000];
y=[Pinst*10^3/Wgto(NR) Pinst*10^3/Wgto(NR)];
line(x,y);
hold on;


%design point
xdesign=ws-0.5;

v=sqrt(2/rho*xdesign*sqrt(k/(cd0)));
ydesign=v/np*(sind(3)+((rho*v*v*cd0)/(2*xdesign))+((2*k)/(rho*v*v))*xdesign);

plot(xdesign,ydesign,'ro');

rear_engine_power = ydesign*Wgto(NR);




%% Outputs
% Rotor related outputs
NR = 4                      % Select the number of rotors;
FOM = FoM(NR)               % Rotor's Figure of Merit
% Mass related outputs
MTOW = Wgto(NR)/g           % Aircraft Gross Take-Off Mass (kg)
Empty_mass = We(NR)/g       % Empty Mass (kg)
Payload = Wpay(NR)/g        % Payload Mass(kg)
Structural_mass = Ws(NR)/g	% Structural Mass (kg)
Porpulsive = Wp(NR)/g       % Propulsive System Mass (kg)
Fuel = Wfuel(NR)            % Fuel Mass (kg)
Batteries = Wbat(NR)        % Mass of batteries (kg)

Rotor_radius_vtol = r(NR)   %raio dos rotores (m)
Installed_power_for_cruise_Kw = Pinst    %Pot�ncia requerida para cruseiro (kw)

P_W=ydesign                 %design point
W_S=xdesign                 %design point

wing_area = Wgto(NR)/xdesign                             %�rea da asa
rear_engine_power_Kw = (ydesign*Wgto(NR)*10^-3)          %Pot�ncia do motor traseiro (Kw)
rear_engine_power_hp = rear_engine_power_Kw * 1.34102209 %Pot�ncia do motor traseiro (hp)


%% 
%Geometric Parameters
hcruise=164; %ft (50m)
vcruiseft=98.4; %ft/s
b=sqrt(AR*wing_area);
mach=vcruiseft/(1036-0.0034*(hcruise-20000))
sweep_angle_LE=0; %fig 4.12 cork
lambda=0.43; %fig 4.10 corke com sweep_angle_LE=0 cork
tc_max=0.1375 %fig 4.5 corke com M=0.09 mas aproximado porque � muito pequeno, nao tem valores (??)

croot=(2*b)/(AR*(1+lambda));
ctip=lambda*croot;
cmean=2/3*croot*(1+lambda+lambda^2)/(1+lambda);

%DRAG
viscosidade_dinamica=1.789e-5;
viscosidade_cinematica=viscosidade_dinamica/rho;     

SwetS=1.977+0.52*tc_max;
Rex=(Vc*cos(sweep_angle_LE)*cmean)/viscosidade_cinematica %sqrt(Rex)<1000 logo laminar

Cf=1.328/sqrt(Rex); %0.455/((log10(Rex))^2.58*(1+0.144*mach^2)^0.65);
xc=0.25;                                                   %%%%MUDAR ESTE VALOR!!!!!!!!!!!!!
F=(1+0.6/xc*tc_max+100*tc_max^4)*(1.34*mach^0.18*(cos(sweep_angle_LE))^0.28);
Q=1; %tabela 4.3
CD0=Cf*F*Q*SwetS

mach_ef=mach*cos(sweep_angle_LE);
beta=sqrt(1-mach_ef^2);
sweep_angle_tc=atan(tan(sweep_angle_LE)-xc*2*croot/b*(1-lambda));
CLa=2*pi*AR/(2+sqrt(4+(AR*beta)^2*(1+(tan(sweep_angle_tc)^2/beta^2))))

a_zerolift=-3*pi/180; %%%%% E ESTE VALOR

CLa0=-CLa*a_zerolift;

for index=1:361
        alpha(index)=(index-181)*pi/180;
        CL(index)=CLa0+CLa*alpha(index);
        CD(index)=CD0+k*CL(index)^2;
end
hold off;
%plot(alpha,CD);

CLairfoil=MTOW*9.81/(0.5*rho*Vc^2*wing_area)

%% Tail sizing (Inverted V-Tail)

% Uniformize variable styles
S = wing_area;
M = mach;

l_fuselage = 2;	% Fuselage length [m]

l_VT = 0.5 * l_fuselage;		% Distance between quarter-chord locations of mean aerodynamic chords of wing and vert stabilizer [m]
C_VT = 0.04;					% Vertical stabilizer scaling coefficient
S_VT = C_VT * (b * S / l_VT);	% Vertical stabilizer area [m^2]

l_HT = l_VT;						% Distance between quarter-chord locations of mean aerodynamic chord of wing and hor stabilizer [m]
C_HT = 0.5;							% Horizontal stabilizer scaling coefficent
S_HT = C_HT * (cmean * S / l_HT);	% Horizontal stabilizer area [m^2]

S_T = S_HT + S_VT;			% Total area of the V-Tail [m^2]
nu = atan(sqrt(S_VT/S_HT));	% Angle between diagonal stabilizer and horizontal plane [rad]
gamma_DA = pi - 2 * nu;		% Dihedral angle between both halves of the V-tail [rad]

%d = 1.1;						% Distance between tails [m]
%b_T = (d / sin(gamma_DA / 2));	% Given the distance d between the tails, we can get the total wingspan through trigonometry [m]
%AR_T = b_T.^2 / S_T;			% Aspect ratio is therefore obtainable
%lambda_T = 0.4;				% Historic data

AR_T = 3;				% Historic data
lambda_T = 0.4;			% Historic data
b_T = sqrt(AR_T * S_T);	% Single wing wingspan [m]

c_root_T = 2 * S_T / (b_T * (1 + lambda_T));								% Root chord for the vertical stabilizer [m]
c_tip_T = lambda_T * c_root_T;												% Tip chord for the vertical stabilizer [m]
c_mean_T = (2/3) * c_root_T * (1 + lambda_T + lambda_T^2) / (1 + lambda_T);	% Mean chord for the vertical stabilizer [m]

if Rex > 1000
	Cf_T = 0.455 / (log10(Rex)).^2.58 * (1 + 0.144 * M.^2).^0.65;	% Friction coefficient for turbulent flow
else
	Cf_T = 1.328 / sqrt(Rex);										% Friction coefficient for laminar flow
end

tc_max_T = 0.12;	% Typical value
xc_T = xc;			% Same as main wing
Q_T = 1.03;			% Historic data for V-tails
Delta_LE_T = 0;		% Sweep angle

S_wet_T = S_T * (1.977 + 0.52 * tc_max_T);
F_T = (1 + 0.6 / xc_T * tc_max_T + 100 * tc_max_T^4) * (1.34 * M^0.18 * (cos(Delta_LE_T))^0.28);

CD_0_T = Cf_T * F_T * Q_T * S_wet_T/S_T;


