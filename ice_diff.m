function out = ice_diff(theta,y,k,B,L,rc,R,N,theta_s, Ach, Ap, m, Rsp, Qin, theta_d, Tinf, Awall)
  %Inputs:
  %theta: independent variable, crankshaft angle [radians]
  %y    : vector of dependent variables, y(1) is pressure [kPa], y(2) is
  %work [kJ/kg]
  %B    : Bore   [m]
  %L    : Stroke [m]
  %rc   : compression ratio [unitless]
  %R    : geometric factor  [l/a]
  %N    : RPM
  %theta_s : combustion start angle [radians]
  %Ach  : crown  head area [m^2]
  %Ap   : piston area      [m^2]
  %m    : mass in one cylinder [kg]
  %Rsp  : specific gas constant [kJ/kg.K]
  %Qin  : heat input per kg mix [kJ/kg mixture]
  %theta_d : combustion duration angle [radians]
  %Tinf : cylinder wall temperature/cooling temperature [K]
  
  P = y(1);
  W = y(2);
  out_geometry = geometry_theta(theta, B, L, rc, R, Ach, Ap, N, 0);
  V  = out_geometry(1); %instantaneous cylinder volume
  dV = out_geometry(2); %dV/d(theta)
  A  = out_geometry(3); %instantaneous cylinder area
  dA = out_geometry(4); %dA/d(theta)
  Up_inst = out_geometry(5); %instantaneous piston velocity
  
  
  a = L/2;
  l = 5.5*a;
  %TASK 2: CALCULATE HEAT LOSS THROUGH MODEL HERE
  Tg   = P*V/m*Rsp;          %gas temperature
  rho  = m/V;                %density
  k_T  = k_diffusion(Tg);    %heat conduction coefficient
  mu_T = mu(Tg);             %viscosity
  Re   = abs(rho*Up_inst*L/mu_T); %Reynolds number
                             %(keep the absolute value in case your
                             %velocity is negative)
  Nu   = 0.244*Re^(0.7);     %Nusselt number
  hg   = (Nu/B)*k_T;         %heat transfer coefficient[W/m^2.K]
  Awall = 3.14*B*(a+l-((l.^2+(a*sin(theta)).^2).^0.5+a*cos(theta))); % cylinder wall area
  Aw   = Ach+Ap+Awall;       %cylinder area
  dQ_theta = hg*Aw*(Tg-Tinf)/N; %dQ/d(theta) due to heat transfer

  %TASK 1: INCLUDE THE FINITE RATE COMBUSTION MODEL HERE
  dQ_comb = 0.0;
  n = 3;
  a = 5;
  xp = 1 - exp(-a*(((theta-theta_s)/theta_d)^n));
  dxp = (n*a/theta_d)*(1-xp)*((theta-theta_s)/theta_d)^(n-1);
  
  if (theta>=theta_s) && (theta<=theta_s+theta_d)
      dQ_comb = m*Qin*dxp %dQ/d(theta) due to combustion
  elseif (theta < theta_s)
      dQ_comb = 0; %dQ/d(theta) at compression stage
  else (theta > theta_s+theta_d)
      dQ_comb = 0; %dQ/d(theta) at expansion stage
  end
##fprintf('    dQ_comb         m     \n')
##fprintf('%10.5f %10.5f %10.5f %10.5f\n',dQ_comb,m)  
  
  %TASKS 1 and 2: YOU WILL NEED TO AUGMENT THE MODEL EQUATION FOR PRESSURE 
  %TO INCLUDE THE TERMS FOR HEAT TRANSFER AND COMBUSTION
  dP = ((k-1)/V)*dQ_comb -k*P/V*dV -dQ_theta;
  dW = P*dV;
  
  out = [dP;dW;dQ_theta];
  
end