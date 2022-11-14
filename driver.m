%Define gas constants
Rsp = 0.287;   %specific gas constant for air [kJ/kg.K]
k  = 1.35;     %specific heat ratio       [unitless]
Cv   = Rsp/(k-1); % specific heat at constant volume       [unitless] 
Cp   = k*Cv;      % specific heat at constant pressure     [unitless]

%Define ambient conditions
Pa = 101.325;         %Ambient pressure          [kPa]
Ta = 333.15;          %Ambient temperature       [K]
rho_a  = Pa/(Rsp*Ta); %Ambient density           [kg/m^3]

%Temperature coming out of the intake pipe
T1 = Ta;       %Initial temperature       [K]

%Engine Geometric Parameters
np = 6;          %Number of cylinders       [unitless]
rc = 10;         %compression ratio         [unitless]
%Specify 2 of the following 3: Vd, B and L
Vd = 0.002;      %Total displacement volume [m^3]
B  = (4*Vd/(np*pi))^(1/3);    %Bore       [m]
L  = 4*Vd/(np*pi*B^2);        %Stroke     [m]
a  = L/2;        %crankshaft length [m]
%Specify either R (geometric ratio) or l (connecting rod length)
R  = 5.5;        %Geometric ratio, l/a      [unitless]
l  = R*a;        %Connecting rod length,    [m]

%Operational parameters
Nstart  = 3000;   %crankshaft rotational speed start[rpm]
Nend    = 5000;   %crankshaft rotational speed end  [rpm]     
Nnumber =    3;   %don't request fewer than 2 or the code will break
theta_s = -20;    %combustion timing angle [degrees]
theta_s = theta_s * pi/180; %convert angle to radians

%Fuel parameters
Qhv = 43400;      %Fuel heating value [kJ/kg fuel]
AF  = 22;         %A/F ratio          [unitless]
Qin = Qhv/(AF+1); %Heat input,        [kJ/kg mixture]
theta_d = 40;     %combustion duration [degrees]
theta_d = theta_d*pi/180; %convert angle to radians

%Cooling parameters
Tinf= 85+273;     %Coolant/Wall Temperature [K]


Vbdc = (Vd/np)*(rc/(rc-1));  %BDC volume [m^3]
Vtdc = (Vd/np)*(1/(rc-1));   %TDC volume [m^3]
Ach  = 0.25*pi*B^2;          %Cylinder head surface area [m^2]
Ap   = Ach;                  %Piston crown  surface area [m^2]

%Intake geometry patterns
LD_intake = 1000; %L/D for intake [unitless]
epsD      = 0.01; %epsilon/D (roughness) of intake [unitless]
BD_intake = 1;    %B/D_intake [unitless]

%Valve Parameters
n_iv      = 2;    %Number of intake valves    [unitless]
n_ev      = 2;    %Number of exhaust valves   [unitless]
Te_min = 273+800; %minimum expected exhaust temperature [K]
N_max  = 8000;    %max crankshaft speed [RPM]
%TASK 3: CALCULATE THE REQUIRED VALVE DIAMETERS HERE
D_iv   = ((4/3.14*(1/n_iv)*1.3*(B^2)*2*L*N_max/60)/((k*R*T1)^0.5))^0.5; %Diameter of intake valves  [m]
D_ev   = ((4/3.14*(1/n_ev)*1.3*(B^2)*2*L*N_max/60)/((k*R*Te_min)^0.5))^0.5; %Diameter of exhaust valves [m]
disp(['Diameter of intake valves = ', num2str(D_iv)])
disp(['Diameter of exhaust valves = ' , num2str(D_ev)])

%Vectors to store the results as a function of RPM
N_results     = linspace(Nstart,Nend,Nnumber);
imep_results  = 0*N_results;
bmep_results  = 0*N_results;
qmep_results  = 0*N_results;
pmep_results  = 0*N_results;
fmep_results  = 0*N_results;
Tb_results    = 0*N_results;
Pb_results    = 0*N_results;
m_results     = 0*N_results;
etav_results  = 0*N_results;
Up_results    = 0*N_results;
DP_im_results = 0*N_results;
isfc_results  = 0*N_results;
bsfc_results  = 0*N_results;
etta_mech_results = 0*N_results;
Ti_results = 0*N_results;
Pi_results = 0*N_results;

i = 1;
for N = Nstart: (Nend-Nstart)/(Nnumber-1) : Nend
    disp(N); %[RPM]
    Up_avg    = 2*L*N/60; %[m/s]
    f_haaland = f_haaland_turbulent(epsD); % % friction factor [unitless]
##    Vintake   = 0.25*Up_avg*np*(B/D_iv)^2;
    %TASK 3: CALCULATE INTAKE MANIFOLD PRESSURE DROP FROM TURBULENT WALL FRICTION
    m_a    = Pa*Vbdc/(Rsp*Ta); %fluid mass in ambient  condition
	  DP_im  = f_haaland*0.5*rho_a*((1.3*m_a*N/rho_a)^2)*LD_intake ; %Pressure drop in intake manifold [kPa]
                                                                   %Vintake = m_dot intake / density intake
                                                                   %m_dot intake = 1.3*m_a*N
##    DP_im  = f_haaland*0.5*rho_a*((Vintake)^2)*LD_intake ;   %Pressure drop in intake manifold [kPa]                                                               
    DP_im_results(i) = DP_im; %store result to plot later
    P_im   = Pa-DP_im;        %update pressure: P_im is the pressure just before the intake valves
##    fprintf('    P_im         Pa         DP_im      \n')    
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P_im,Pa,DP_im)
    pmep   = DP_im + DP_iv + DP_ev + DP_es; %pump mean effective pressure 
    %TASK 3: CALCULATE INTAKE VALVE PRESURE DROP DP_iv,
    %      EXHAUST VALVE PRESSURE DROP DP_ev,
    %      EXHAUST SYSTEM PRESSURE DROP DP_es
    Ces    = 0.178; %prportionality constant
    DP_iv  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_iv*D_iv^2)))^2;  %pressure drop at intake  valve [kPa]    
    DP_ev  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_ev*D_ev^2)))^2;  %pressure drop at exhaust valve [kPa]
    DP_es  = Ces*((P_im/Pa)*Up_avg)^2;                            %exhaust system pressure drop   [kPa]
    P1     = P_im - DP_iv; %update pressure: P1 is the pressure after the intake valve, before compression
##    fprintf('    P1         P_im         DP_iv      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P_im,DP_iv)
    pmep   = DP_im + DP_iv + DP_ev + DP_es; %pump mean effective pressure 

    m      = P1*Vbdc/(Rsp*T1); %mass in one cylinder [kg]
##    fprintf('    P1         Vbdc         Rsp         T1      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,Vbdc,Rsp,T1)
    %integrate the cycle
    %The function integration_driver splits the cycle in 3 parts:
    %compression, combustion, expansion and performs three separate
    %integrations, one for each part.
    [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd/np, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im);
    
    %store the results in the vectors for plotting
    Ti_results(i)   = Ti;
    Pi_results(i)   = Pi;
    imep_results(i) = imep;
    bmep_results(i) = bmep-pmep;
    Tb_results(i)   = Tb;
    Pb_results(i)   = Pb;
    qmep_results(i) = qmep;
    pmep_results(i) = pmep;
    fmep_results(i) = fmep;
    m_results(i)    = m;
    etav_results(i) = P1/(Rsp*T1*rho_a);
    Up_results(i)   = Up_avg;
    isfc_results(i) = isfc;
    bsfc_results(i) = bsfc;
    etta_mech_results(i) = etta_mech;
    i=i+1;
end    

plot results after calculations are done for all values of N
figure
plot(N_results, bmep_results, '-b', N_results, imep_results, '-r')
title('MEP')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('MEP [kPa]')
legend('BMEP', 'IMEP')

plot(N_results, fmep_results, '-b', N_results, qmep_results, '-r', N_results, pmep_results, '-g')
title('MEP')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('MEP [kPa]')
legend('FMEP', 'QMEP', 'PMEP')

figure
plot(N_results, Ti_results, '-')
title('Indicated Torque')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Indicated Torque [kN.m]')

figure
plot(N_results, Pi_results, '-')
title('Indicated Power')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Indicated Power [kW]')

figure
plot(N_results, Tb_results, '-')
title('Brake Torque')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Brake Torque [kN.m]')

figure
plot(N_results, Pb_results, '-')
title('Brake Power')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Brake Power [kW]')

figure
plot(N_results, etav_results, '-')
title('Volumetric Efficiency')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Volumetric EFficiency')

figure
plot(N_results, Up_results, '-')
title('Average Piston Velocity')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('Velocity')

figure
plot(N_results, isfc_results, '-')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('isfc')

figure
plot(N_results, bsfc_results, '-')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('bsfc')

figure
plot(N_results, etta_mech_results, '-')
xlabel('Crankshaft Rotational Speed [RPM]')
ylabel('mechanical efficiency')

% theta_s variation effect on engine
%Operational parameters
Thetasstart  = -40*(pi/180);   
Thetasend    = 10*(pi/180);     
Thetasnumber = 100;  %don't request fewer than 2 or the code will break

%Vectors to store the results as a function of Theta
Thetas_results = linspace(Thetasstart,Thetasend,Thetasnumber);
imep_results  = 0*Thetas_results ;
bmep_results  = 0*Thetas_results ;
qmep_results  = 0*Thetas_results;
pmep_results  = 0*Thetas_results;
fmep_results  = 0*Thetas_results;
Tb_results    = 0*Thetas_results;
Pb_results    = 0*Thetas_results;
m_results     = 0*Thetas_results;
etav_results  = 0*Thetas_results;
Up_results    = 0*Thetas_results;
DP_im_results = 0*Thetas_results;
isfc_results  = 0*Thetas_results;
bsfc_results  = 0*Thetas_results;
etta_mech_results = 0*Thetas_results;
Ti_results = 0*Thetas_results;
Pi_results = 0*Thetas_results;

i = 1;
for theta_s = Thetasstart: (Thetasend-Thetasstart)/(Thetasnumber-1) : Thetasend
    disp(theta_s); %[rad]
    N = 4000; %[RPM]
    Up_avg    = 2*L*N/60; %[m/s]
    f_haaland = f_haaland_turbulent(epsD); % friction factor [unitless] 
##    Vintake   = 0.25*Up_avg*np*(B/D_iv)^2;
    %TASK 3: CALCULATE INTAKE MANIFOLD PRESSURE DROP FROM TURBULENT WALL FRICTION
    m_a    = Pa*Vbdc/(Rsp*Ta); %fluid mass in ambient  condition
	  DP_im  = f_haaland*0.5*rho_a*((1.3*m_a*N/rho_a)^2)*LD_intake ; %Pressure drop in intake manifold [kPa]
                                                                   %Vintake = m_dot intake / density intake
                                                                   %m_dot intake = 1.3*m_a*N
##    DP_im  = f_haaland*0.5*rho_a*((Vintake)^2)*LD_intake ;   %Pressure drop in intake manifold [kPa]                                                               
    DP_im_results(i) = DP_im; %store result to plot later
    P_im   = Pa-DP_im;        %update pressure: P_im is the pressure just before the intake valves
##    fprintf('    P_im         Pa         DP_im      \n')    
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P_im,Pa,DP_im)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;   %pump mean effective pressure
    %TASK 3: CALCULATE INTAKE VALVE PRESURE DROP DP_iv,
    %      EXHAUST VALVE PRESSURE DROP DP_ev,
    %      EXHAUST SYSTEM PRESSURE DROP DP_es
    Ces    = 0.178;  %prportionality constant
    DP_iv  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_iv*D_iv^2)))^2;  %pressure drop at intake  valve [kPa]  
    DP_ev  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_ev*D_ev^2)))^2;  %pressure drop at exhaust valve [kPa]
    DP_es  = Ces*((P_im/Pa)*Up_avg)^2; %exhaust system pressure drop  [kPa]
    P1     = P_im - DP_iv; %update pressure: P1 is the pressure after the intake valve, before compression
##    fprintf('    P1         P_im         DP_iv      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P_im,DP_iv)
    pmep   = DP_im + DP_iv + DP_ev + DP_es; %pump mean effective pressure
    m      = P1*Vbdc/(Rsp*T1); %mass in one cylinder [kg]
##    fprintf('    P1         Vbdc         Rsp         T1      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,Vbdc,Rsp,T1)
    %integrate the cycle
    %The function integration_driver splits the cycle in 3 parts:
    %compression, combustion, expansion and performs three separate
    %integrations, one for each part.
    [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd/np, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im);
    
    %store the results in the vectors for plotting
    Ti_results(i)   = Ti;
    Pi_results(i)   = Pi;
    imep_results(i) = imep;
    bmep_results(i) = bmep-pmep;
    Tb_results(i)   = Tb;
    Pb_results(i)   = Pb;
    qmep_results(i) = qmep;
    pmep_results(i) = pmep;
    fmep_results(i) = fmep;
    m_results(i)    = m;
    etav_results(i) = P1/(Rsp*T1*rho_a);
    Up_results(i)   = Up_avg;
    isfc_results(i) = isfc;
    bsfc_results(i) = bsfc;
    etta_mech_results(i) = etta_mech;
    i=i+1;
end    

%plot results after calculations are done for all values of N
figure
plot(Thetas_results/(pi/180), bmep_results, '-b', Thetas_results/(pi/180), imep_results, '-r')
title('MEP')
xlabel('Theta spark (degree)')
ylabel('MEP [kPa]')
legend('BMEP', 'IMEP')

plot(Thetas_results/(pi/180), fmep_results, '-b', Thetas_results/(pi/180), qmep_results, '-r', Thetas_results/(pi/180), pmep_results, '-g')
title('MEP')
xlabel('Theta spark (degree)')
ylabel('MEP [kPa]')
legend('FMEP', 'QMEP', 'PMEP')

figure
plot(Thetas_results/(pi/180), Ti_results, '-')
title('Indicated Torque')
xlabel('Theta spark (degree)]')
ylabel('Indicated Torque [kN.m]')

figure
plot(Thetas_results/(pi/180), Pi_results, '-')
title('Indicated Power')
xlabel('Theta spark (degree)]')
ylabel('Indicated Power [kW]')

figure
plot(Thetas_results/(pi/180), Tb_results, '-')
title('Brake Torque')
xlabel('Theta spark (degree)]')
ylabel('Brake Torque [kN.m]')

figure
plot(Thetas_results/(pi/180), Pb_results, '-')
title('Brake Power')
xlabel('Theta spark (degree)]')
ylabel('Brake Power [kW]')

figure
plot(Thetas_results/(pi/180), etav_results, '-')
title('Volumetric Efficiency')
xlabel('Theta spark (degree)]')
ylabel('Volumetric EFficiency')

figure
plot(Thetas_results/(pi/180), Up_results, '-')
title('Average Piston Velocity')
xlabel('Theta spark (degree)]')
ylabel('Velocity')

figure
plot(Thetas_results/(pi/180), isfc_results, '-')
xlabel('Theta spark (degree)]')
ylabel('isfc')

figure
plot(Thetas_results/(pi/180), bsfc_results, '-')
xlabel('Theta spark (degree)]')
ylabel('bsfc')

figure
plot(Thetas_results/(pi/180), etta_mech_results, '-')
xlabel('Theta spark (degree)]')
ylabel('mechanical efficiency')

% theta_d variation effect on engine
%Operational parameters
Theta_start  = 30*(pi/180);   
Theta_end    = 50*(pi/180);     
Theta_number = 100;  %don't request fewer than 2 or the code will break

%Vectors to store the results as a function of RPM
Theta_results = linspace(Theta_start,Theta_end,Theta_number);
imep_results  = 0*Theta_results ;
bmep_results  = 0*Theta_results ;
qmep_results  = 0*Theta_results;
pmep_results  = 0*Theta_results;
fmep_results  = 0*Theta_results;
Tb_results    = 0*Theta_results;
Pb_results    = 0*Theta_results;
m_results     = 0*Theta_results;
etav_results  = 0*Theta_results;
Up_results    = 0*Theta_results;
DP_im_results = 0*Theta_results;
isfc_results  = 0*Theta_results;
bsfc_results  = 0*Theta_results;
etta_mech_results = 0*Theta_results;
Ti_results = 0*Theta_results;
Pi_results = 0*Theta_results;

i = 1;
for theta_d = Theta_start: (Theta_end-Theta_start)/(Theta_number-1) : Theta_end
    disp(theta_d);
    N = 4000;
    Up_avg    = 2*L*N/60;
    f_haaland = f_haaland_turbulent(epsD);
##    Vintake   = 0.25*Up_avg*np*(B/D_iv)^2;
    %TASK 3: CALCULATE INTAKE MANIFOLD PRESSURE DROP FROM TURBULENT WALL FRICTION
    m_a    = Pa*Vbdc/(Rsp*Ta); %fluid mass in ambient  condition
	  DP_im  = f_haaland*0.5*rho_a*((1.3*m_a*N/rho_a)^2)*LD_intake ; %Pressure drop in intake manifold [kPa]
                                                                   %Vintake = m_dot intake / density intake
                                                                   %m_dot intake = 1.3*m_a*N
##    DP_im  = f_haaland*0.5*rho_a*((Vintake)^2)*LD_intake ;   %Pressure drop in intake manifold [kPa]                                                               
    DP_im_results(i) = DP_im; %store result to plot later
    P_im   = Pa-DP_im;        %update pressure: P_im is the pressure just before the intake valves
##    fprintf('    P_im         Pa         DP_im      \n')    
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P_im,Pa,DP_im)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;  %pump mean effective pressure 
    %TASK 3: CALCULATE INTAKE VALVE PRESURE DROP DP_iv,
    %      EXHAUST VALVE PRESSURE DROP DP_ev,
    %      EXHAUST SYSTEM PRESSURE DROP DP_es
    Ces    = 0.178; %prportionality constant
    DP_iv  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_iv*D_iv^2)))^2;  %pressure drop at intake  valve [kPa]    
    DP_ev  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_ev*D_ev^2)))^2;  %pressure drop at exhaust valve [kPa]
    DP_es  = Ces*((P_im/Pa)*Up_avg)^2;  %exhaust system pressure drop  [kPa]
    P1     = P_im - DP_iv; %update pressure: P1 is the pressure after the intake valve, before compression
##    fprintf('    P1         P_im         DP_iv      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P_im,DP_iv)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;  %pump mean effective pressure

    m      = P1*Vbdc/(Rsp*T1); %mass in one cylinder [kg]
##    fprintf('    P1         Vbdc         Rsp         T1      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,Vbdc,Rsp,T1)
    %integrate the cycle
    %The function integration_driver splits the cycle in 3 parts:
    %compression, combustion, expansion and performs three separate
    %integrations, one for each part.
    [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd/np, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im);
    
    %store the results in the vectors for plotting
    Ti_results(i)   = Ti;
    Pi_results(i)   = Pi;
    imep_results(i) = imep;
    bmep_results(i) = bmep-pmep;
    Tb_results(i)   = Tb;
    Pb_results(i)   = Pb;
    qmep_results(i) = qmep;
    pmep_results(i) = pmep;
    fmep_results(i) = fmep;
    m_results(i)    = m;
    etav_results(i) = P1/(Rsp*T1*rho_a);
    Up_results(i)   = Up_avg;
    isfc_results(i) = isfc;
    bsfc_results(i) = bsfc;
    etta_mech_results(i) = etta_mech;
    i=i+1;
end    

%plot results after calculations are done for all values of N
figure
plot(Theta_results/(pi/180), bmep_results, '-b', Theta_results/(pi/180), imep_results, '-r')
title('MEP')
xlabel('Theta_d (degree)')
ylabel('MEP [kPa]')
legend('BMEP', 'IMEP')

plot(Theta_results/(pi/180), fmep_results, '-b', Theta_results/(pi/180), qmep_results, '-r', Theta_results/(pi/180), pmep_results, '-g')
title('MEP')
xlabel('Theta_d (degree)')
ylabel('MEP [kPa]')
legend('FMEP', 'QMEP', 'PMEP')

figure
plot(Theta_results/(pi/180), Ti_results, '-')
title('Indicated Torque')
xlabel('Theta_d (degree)]')
ylabel('Indicated Torque [kN.m]')

figure
plot(Theta_results/(pi/180), Pi_results, '-')
title('Indicated Power')
xlabel('Theta_d (degree)]')
ylabel('Indicated Power [kW]')

figure
plot(Theta_results/(pi/180), Tb_results, '-')
title('Brake Torque')
xlabel('Theta_d (degree)]')
ylabel('Brake Torque [kN.m]')

figure
plot(Theta_results/(pi/180), Pb_results, '-')
title('Brake Power')
xlabel('Theta_d (degree)]')
ylabel('Brake Power [kW]')

figure
plot(Theta_results/(pi/180), etav_results, '-')
title('Volumetric Efficiency')
xlabel('Theta_d (degree)]')
ylabel('Volumetric EFficiency')

figure
plot(Theta_results/(pi/180), Up_results, '-')
title('Average Piston Velocity')
xlabel('Theta_d (degree)]')
ylabel('Velocity')

figure
plot(Theta_results/(pi/180), isfc_results, '-')
xlabel('Theta_d (degree)]')
ylabel('isfc')

figure
plot(Theta_results/(pi/180), bsfc_results, '-')
xlabel('Theta_d (degree)]')
ylabel('bsfc')

figure
plot(Theta_results/(pi/180), etta_mech_results, '-')
xlabel('Theta_d (degree)]')
ylabel('mechanical efficiency')

% valve size variation effect on engine
%Operational parameters
percentage_start  = 0.8;   
percentage_end    = 1.2;     
Dvision_number = 100;  %don't request fewer than 2 or the code will break

%Vectors to store the results as a function of RPM
percentage_results = linspace(percentage_start,percentage_end,Dvision_number);
imep_results  = 0*percentage_results ;
bmep_results  = 0*percentage_results ;
qmep_results  = 0*percentage_results;
pmep_results  = 0*percentage_results;
fmep_results  = 0*percentage_results;
Tb_results    = 0*percentage_results;
Pb_results    = 0*percentage_results;
m_results     = 0*percentage_results;
etav_results  = 0*percentage_results;
Up_results    = 0*percentage_results;
DP_im_results = 0*percentage_results;
isfc_results  = 0*percentage_results;
bsfc_results  = 0*percentage_results;
etta_mech_results = 0*percentage_results;
Ti_results = 0*percentage_results;
Pi_results = 0*percentage_results;

i = 1;
for percentage = percentage_start: (percentage_end-percentage_start)/(Dvision_number-1) : percentage_end
    disp(percentage);
    N = 4000;
    D_iv = percentage*D_iv;
    D_ev = percentage*D_ev;
    Up_avg    = 2*L*N/60;
    f_haaland = f_haaland_turbulent(epsD);
##    Vintake   = 0.25*Up_avg*np*(B/D_iv)^2;
    %TASK 3: CALCULATE INTAKE MANIFOLD PRESSURE DROP FROM TURBULENT WALL FRICTION
    m_a    = Pa*Vbdc/(Rsp*Ta); %fluid mass in ambient  condition
	  DP_im  = f_haaland*0.5*rho_a*((1.3*m_a*N/rho_a)^2)*LD_intake ; %Pressure drop in intake manifold [kPa]
                                                                   %Vintake = m_dot intake / density intake
                                                                   %m_dot intake = 1.3*m_a*N
##    DP_im  = f_haaland*0.5*rho_a*((Vintake)^2)*LD_intake ;   %Pressure drop in intake manifold [kPa]                                                               
    DP_im_results(i) = DP_im; %store result to plot later
    P_im   = Pa-DP_im;        %update pressure: P_im is the pressure just before the intake valves
##    fprintf('    P_im         Pa         DP_im      \n')    
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P_im,Pa,DP_im)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;   %pump mean effective pressure
    %TASK 3: CALCULATE INTAKE VALVE PRESURE DROP DP_iv,
    %      EXHAUST VALVE PRESSURE DROP DP_ev,
    %      EXHAUST SYSTEM PRESSURE DROP DP_es
    Ces    = 0.178;  %prportionality constant
    DP_iv  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_iv*D_iv^2)))^2;  %pressure drop at intake  valve [kPa]     
    DP_ev  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_ev*D_ev^2)))^2;  %pressure drop at exhaust valve [kPa]
    DP_es  = Ces*((P_im/Pa)*Up_avg)^2;  %exhaust system pressure drop  [kPa]
    P1     = P_im - DP_iv; %update pressure: P1 is the pressure after the intake valve, before compression
##    fprintf('    P1         P_im         DP_iv      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P_im,DP_iv)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;  %pump mean effective pressure

    m      = P1*Vbdc/(Rsp*T1); %mass in one cylinder [kg]
##    fprintf('    P1         Vbdc         Rsp         T1      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,Vbdc,Rsp,T1)
    %integrate the cycle
    %The function integration_driver splits the cycle in 3 parts:
    %compression, combustion, expansion and performs three separate
    %integrations, one for each part.
    [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd/np, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im);
    
    %store the results in the vectors for plotting
    Ti_results(i)   = Ti;
    Pi_results(i)   = Pi;
    imep_results(i) = imep;
    bmep_results(i) = bmep-pmep;
    Tb_results(i)   = Tb;
    Pb_results(i)   = Pb;
    qmep_results(i) = qmep;
    pmep_results(i) = pmep;
    fmep_results(i) = fmep;
    m_results(i)    = m;
    etav_results(i) = P1/(Rsp*T1*rho_a);
    Up_results(i)   = Up_avg;
    isfc_results(i) = isfc;
    bsfc_results(i) = bsfc;
    etta_mech_results(i) = etta_mech;
    i=i+1;
end    

%plot results after calculations are done for all values of N
figure
plot(percentage_results*100, bmep_results, '-b', percentage_results*100, imep_results, '-r')
title('MEP')
xlabel('Percentage')
ylabel('MEP [kPa]')
legend('BMEP', 'IMEP')

plot(percentage_results*100, fmep_results, '-b', percentage_results*100, qmep_results, '-r', percentage_results*100, pmep_results, '-g')
title('MEP')
xlabel('Percentage')
ylabel('MEP [kPa]')
legend('FMEP', 'QMEP', 'PMEP')

figure
plot(percentage_results*100, Ti_results, '-')
title('Indicated Torque')
xlabel('Percentage')
ylabel('Indicated Torque [kN.m]')

figure
plot(percentage_results*100, Pi_results, '-')
title('Indicated Power')
xlabel('Percentage')
ylabel('Indicated Power [kW]')

figure
plot(percentage_results*100, Tb_results, '-')
title('Brake Torque')
xlabel('Percentage')
ylabel('Brake Torque [kN.m]')

figure
plot(percentage_results*100, Pb_results, '-')
title('Brake Power')
xlabel('Percentage')
ylabel('Brake Power [kW]')

figure
plot(percentage_results*100, etav_results, '-')
title('Volumetric Efficiency')
xlabel('Percentage')
ylabel('Volumetric EFficiency')

figure
plot(percentage_results*100, Up_results, '-')
title('Average Piston Velocity')
xlabel('Percentage')
ylabel('Velocity')

figure
plot(percentage_results*100, isfc_results, '-')
xlabel('Percentage')
ylabel('isfc')

figure
plot(percentage_results*100, bsfc_results, '-')
xlabel('Percentage')
ylabel('bsfc')

figure
plot(percentage_results*100, etta_mech_results, '-')
xlabel('Percentage')
ylabel('mechanical efficiency')

%compression ration (rc) variation effect on engine
%Operational parameters
percentage_start = 0.7;   
percentage_end   = 1.3;     
Dvision_number   = 100;  %don't request fewer than 2 or the code will break

%Vectors to store the results as a function of RPM
percentage_results = linspace(percentage_start,percentage_end,Dvision_number);
imep_results  = 0*percentage_results ;
bmep_results  = 0*percentage_results ;
qmep_results  = 0*percentage_results;
pmep_results  = 0*percentage_results;
fmep_results  = 0*percentage_results;
Tb_results    = 0*percentage_results;
Pb_results    = 0*percentage_results;
m_results     = 0*percentage_results;
etav_results  = 0*percentage_results;
Up_results    = 0*percentage_results;
DP_im_results = 0*percentage_results;
isfc_results  = 0*percentage_results;
bsfc_results  = 0*percentage_results;
etta_mech_results = 0*percentage_results;
Ti_results = 0*percentage_results;
Pi_results = 0*percentage_results;

i = 1;
for percentage = percentage_start: (percentage_end-percentage_start)/(Dvision_number-1) : percentage_end
    disp(percentage);
    N = 4000;
##    rc = percentage*rc;
    Vd = percentage*Vd;
    Up_avg    = 2*L*N/60;
    f_haaland = f_haaland_turbulent(epsD);
##    Vintake   = 0.25*Up_avg*np*(B/D_iv)^2;
    %TASK 3: CALCULATE INTAKE MANIFOLD PRESSURE DROP FROM TURBULENT WALL FRICTION
    m_a    = Pa*Vbdc/(Rsp*Ta); %fluid mass in ambient  condition
	  DP_im  = f_haaland*0.5*rho_a*((1.3*m_a*N/rho_a)^2)*LD_intake ; %Pressure drop in intake manifold [kPa]
                                                                   %Vintake = m_dot intake / density intake
                                                                   %m_dot intake = 1.3*m_a*N
##    DP_im  = f_haaland*0.5*rho_a*((Vintake)^2)*LD_intake ;   %Pressure drop in intake manifold [kPa]                                                               
    DP_im_results(i) = DP_im; %store result to plot later
    P_im   = Pa-DP_im;        %update pressure: P_im is the pressure just before the intake valves
##    fprintf('    P_im         Pa         DP_im      \n')    
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P_im,Pa,DP_im)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;   %pump mean effective pressure
    %TASK 3: CALCULATE INTAKE VALVE PRESURE DROP DP_iv,
    %      EXHAUST VALVE PRESSURE DROP DP_ev,
    %      EXHAUST SYSTEM PRESSURE DROP DP_es
    Ces    = 0.178;  %prportionality constant
    DP_iv  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_iv*D_iv^2)))^2;  %pressure drop at intake  valve [kPa]    
    DP_ev  = 0.00412*((P_im/Pa)*((Up_avg*B^2)/(n_ev*D_ev^2)))^2;  %pressure drop at exhaust valve [kPa]]
    DP_es  = Ces*((P_im/Pa)*Up_avg)^2;  %exhaust system pressure drop  [kPa]
    P1     = P_im - DP_iv; %update pressure: P1 is the pressure after the intake valve, before compression
##    fprintf('    P1         P_im         DP_iv      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P_im,DP_iv)
    pmep   = DP_im + DP_iv + DP_ev + DP_es;  %pump mean effective pressure

    m      = P1*Vbdc/(Rsp*T1); %mass in one cylinder [kg]
##    fprintf('    P1         Vbdc         Rsp         T1      \n')
##    fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,Vbdc,Rsp,T1)
    %integrate the cycle
    %The function integration_driver splits the cycle in 3 parts:
    %compression, combustion, expansion and performs three separate
    %integrations, one for each part.
    [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd/np, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im);
    
    %store the results in the vectors for plotting
    Ti_results(i)   = Ti;
    Pi_results(i)   = Pi;
    imep_results(i) = imep;
    bmep_results(i) = bmep-pmep;
    Tb_results(i)   = Tb;
    Pb_results(i)   = Pb;
    qmep_results(i) = qmep;
    pmep_results(i) = pmep;
    fmep_results(i) = fmep;
    m_results(i)    = m;
    etav_results(i) = P1/(Rsp*T1*rho_a);
    Up_results(i)   = Up_avg;
    isfc_results(i) = isfc;
    bsfc_results(i) = bsfc;
    etta_mech_results(i) = etta_mech;
    i=i+1;
end    

%plot results after calculations are done for all values of N
figure
plot(percentage_results*0.002, bmep_results, '-b', percentage_results*0.002, imep_results, '-r')
title('MEP')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('MEP [kPa]')
legend('BMEP', 'IMEP')

plot(percentage_results*0.002, fmep_results, '-b', percentage_results*0.002, qmep_results, '-r', percentage_results*0.002, pmep_results, '-g')
title('MEP')
##xlabel('Percentage')
xlabel('total displacement volume (Vd) m^3')
ylabel('MEP [kPa]')
legend('FMEP', 'QMEP', 'PMEP')

figure
plot(percentage_results*0.002, Ti_results, '-')
title('Indicated Torque')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Indicated Torque [kN.m]')

figure
plot(percentage_results*0.002, Pi_results, '-')
title('Indicated Power')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Indicated Power [kW]')

figure
plot(percentage_results*0.002, Tb_results, '-')
title('Brake Torque')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Brake Torque [kN.m]')

figure
plot(percentage_results*0.002, Pb_results, '-')
title('Brake Power')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Brake Power [kW]')

figure
plot(percentage_results*0.002, etav_results, '-')
title('Volumetric Efficiency')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Volumetric Efficiency')

figure
plot(percentage_results*0.002, Up_results, '-')
title('Average Piston Velocity')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('Velocity')

figure
plot(percentage_results*0.002, isfc_results, '-')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('isfc')

figure
plot(percentage_results*0.002, bsfc_results, '-')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('bsfc')

figure
plot(percentage_results*0.002, etta_mech_results, '-')
##xlabel('Compression ratio')
xlabel('total displacement volume (Vd) m^3')
ylabel('mechanical efficiency')