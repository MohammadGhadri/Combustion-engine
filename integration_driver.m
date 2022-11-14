function [Ti,Pi,imep,fmep,bmep,Tb,Pb,qmep,theta,P,V,isfc,bsfc,etta_mech] = integration_driver(Pa, P1, T1, Vd, np, B, R, k, rc, N, theta_s, Rsp, Qin, theta_d, L, Vbdc, Ach, Ap, m, Cv, Cp, Tinf, Up_avg, P_im)
%%integration driver
%inputs:
%Pa : Ambient pressure          [kPa]
%P1 : Initial pressure          [kPa]
%T1 : Initial temperature       [K]
%Vd : Total displacement volume [m^3]
%np : Number of cylinders       [unitless]
%B  : Bore                      [m]
%R  : Geometric ratio, l/a      [unitless]
%k  : specific heat ratio       [unitless]
%rc : compression ratio         [unitless]
%N  : crankshaft rotational speed        [rpm]
%theta_s : combustion start timing angle [radians]
%Rsp : specific gas constant             [kJ/kg.K]
%Qin : Heat input                        [kJ/kg mixture]
%theta_d : combustion duration angle     [radians]
%L   : Stroke               [m]
%Vbdc: BDC volume           [m^3]
%Ach : Cylinder head area   [m^2]
%Ap  : Piston crown area    [m^2]
%m   : mass in the cylinder [kg]
%Cv  : Specific heat at constant volume   [kJ/kg.K]
%Cp  : Specific heat at constant pressure [kJ/kg.K]
%Up_avg : Piston average speed
%P_im: Intake manifold pressure

%Integrate compression to spark  
ICs   = [ P1, 0, 0]; %P1, W=0, Q=0
tspan = [-pi,theta_s]; %Span of angle, compression
[t,y] = ode45(@(t,y) ice_diff(t,y,k,B,L,rc,R,N,theta_s, Ach, Ap, m, Rsp, Qin, theta_d, Tinf), tspan,ICs );
out_geometry = geometry_theta(t, B, L, rc, R, Ach, Ap, N, 0);
theta_comp = t;
P_comp     = y(:,1);
V_comp     = out_geometry(:,1);
V2 = V_comp(end);
P2 = y(end,1);
W2 = y(end,2);
Q2 = y(end,3);
T2 = P2*V2/(m*Rsp); 

%Integrate combustion phase
%Note for task 1: remember to change the limits of integration so that you
%perform the integration in 3 steps:
%(1) -pi to combustion start
%(2) combustion start to combustion end
%(3) combustion end to pi
%%Instantanous combustion model
%V3 = V2;
%T3 = T2 + Qin/Cv;
%P3 = P2* T3/T2;
%W3 = W2;
%Q3 = Q2;
%theta_comb = [0;0];
%P_comb     = [P2;P3];
%V_comb     = [V2;V3];
ICs   = [ P2, W2, Q2]; %P1, W=W_comb, Q=Q_comb
tspan = [theta_s,theta_s+theta_d]; %Span of angle, combustion
[t,y] = ode45(@(t,y) ice_diff(t,y,k,B,L,rc,R,N,theta_s, Ach, Ap, m, Rsp, Qin, theta_d, Tinf), tspan,ICs);
out_geometry = geometry_theta(t, B, L, rc, R, Ach, Ap, N, 0);
theta_comb = t;
P_comb = y(:,1);
V_comb = out_geometry(:,1);
V3 = V_comb(end);
P3 = y(end,1);
W3 = y(end,2);
Q3 = y(end,3);
T3 = P3*V3/(m*Rsp); 

%Integrate expansion phase
ICs   = [ P3, W3, Q3]; %P1, W=W_comp, Q=Q_comp
tspan = [theta_s+theta_d, pi]; %Span of angle, expansion
[t,y] = ode45(@(t,y) ice_diff(t,y,k,B,L,rc,R,N,theta_s, Ach, Ap, m, Rsp, Qin, theta_d, Tinf), tspan,ICs );
out_geometry = geometry_theta(t, B, L, rc, R, Ach, Ap, N, 0);
theta_exp = t;
P_exp     = y(:,1);
V_exp     = out_geometry(:,1);
V4 = V_exp(end);
P4 = y(end,1);
W4 = y(end,2);
Q4 = y(end,3);
T4 = P4*V4/(m*Rsp);

%PRINT THE TEMPERATURE AND PRESSURE AT 4 MAIN STATES
%T1,P1: initial temperature and pressure before compression
%T2,P2: T and P before start of combustion
%T3,P3: T and P after end of combustion
%T4,P4: T and P at the end of the cycle
##fprintf('    T1         T2         T3         T4      \n')
##fprintf('%10.5f %10.5f %10.5f %10.5f\n',T1,T2,T3,T4)
##fprintf('    P1         P2         P3         P4      \n')
##fprintf('%10.5f %10.5f %10.5f %10.5f\n',P1,P2,P3,P4)

Wi_total    = np*W4;
Qloss_total = np*Q4;
%Compute imep, indicated torque and indicated power
Ti = Wi_total/(4*pi);   %indicated torque [kN.m]
Pi = (N/60)*Wi_total/2; %indicated power  [kW]
imep = Wi_total/Vd;     %imep             [kPa]
qmep = Qloss_total/Vd;  %qmep             [kPa]

%TASK 4: IMPLEMENT FRICTION LOSSES HERE
%CONSIDER SKIRT, RING AND GAS LOADING
%Skirt Friction
Cps = 0.294;
fmep_skirt = Cps*(Up_avg/B); % [kPa]

%Ring Friction
Cpr = 4.06/100;
fmep_rings = Cpr*(1+1000/N)*(1/B^2); % [kPa]

%Gas Loading
Cg = 6.89;
k_g = 2.38/100;
fmep_gas = Cg*(P_im/Pa)*(0.088*rc+0.182*rc^(1.33-k_g*Up_avg)); % [kPa]

fmep = fmep_skirt + fmep_rings + fmep_gas;  % [kPa]
bmep = imep - fmep;                         % [kPa]
Tb   = bmep * Vd/(4*pi);                    % [kN.m]
Pb   = (N/60)* bmep * Vd/2;                 % [kW]

%Assemble Output Data
theta = [theta_comp;theta_comb;theta_exp];
P     = [P_comp;P_comb;P_exp];
V     = [V_comp;V_comb;V_exp];

isfc = 1.3*m*N*np/Pi; %m_dot = ((m/(AF+1))/2)*N/60*3600 ~ 1.3*m*N 
bsfc = 1.3*m*N*np/Pb;
etta_mech = (Pb/Pi)*100;

%if you uncomment the following blocks, the function will plot
%the P(theta), V(theta) and P-V plots for each value of N
%You probably want to select a low number of N in driver to not
%be overwhelmed by plots.
%Something like start = 1000, stop = 3000, N = 2

 figure
 plot(theta,P)
 title(['N = ',num2str(N)])
 xlabel('Crank Angle [rad]')
 ylabel('Pressure [kPa]')
 
 figure
 plot(theta,V)
 title(['N = ',num2str(N)])
 xlabel('Crank Angle [rad]')
 ylabel('Volume [m^3]')
 
 figure
 plot(V,P)
 title(['N = ',num2str(N)]) 
 xlabel('Volume [m^3]')
 ylabel('Pressure [kPa]')
 disp(['RPMs = ' , num2str(N)])
 disp(['Total Work = ' , num2str(Wi_total)])
