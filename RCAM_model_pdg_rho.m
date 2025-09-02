function[XDOT]=RCAM_model_pdg_rho(X,U,rho_h)

%% MODELO RCAM Similar BOEING 757-200 <-- GARTEUR 15TH JUNE 1995

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     1º INSERTAR A CONTINUACIÓN PARÁMETROS AERONAVE DE ESTUDIO:        %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------  Saturación implementada en simulink -------------------

m   = 120000;              % masa total (cte) (kg)
mac = 6.6;                % MAC (m)
lt   = 24.8;               % Brazo AC tail <--> CG A/C (m)
S    = 260;                % Wing platform area (m^2)
St   = 64;                 % Tail platform area (m^2)

% Matriz Inercia
Ib=m*[40.07 0 -2.0923;0 64 0;-2.0923 0 99.92];
invIb = inv(Ib);

% Posición CG en metros
Xcg = 0.23*mac;
Ycg = 0;        
Zcg = 0.10*mac;

% Posición AC en metros
Xac = 0.12*mac;         
Yac = 0;                 
Zac = 0;                 

% Posición de aplicación del empuje en metros
% Motor 1
Xapt1 = 0;    
Yapt1 = -7.94;
Zapt1 = -1.9; 
% Motor
Xapt2 = 0;    
Yapt2 = 7.94; 
Zapt2 = -1.9; 

% Parámetros aerodinámicos Coeficientes sustentación
depsda       = 0.25;             % Downwash (adim)
alpha_L0     = -11.5*pi/180;     % Alpha sustentación nula (rad)
n            = 5.5;              % Pendiente tramo linear CL
a3           = -768.5;           % Coeficientes curva CL-alfa --> alpha^3
a2           = 609.2;            % Coeficientes curva CL-alfa --> alpha^2
a1           = -155.2;           % Coeficientes curva CL-alfa --> alpha^1
a0           = 15.212;           % Coeficientes curva CL-alfa --> alpha
alpha_switch = 14.5*(pi/180);    % Alpha fin tramo linear (rad)


%% 2º VECTORES DE ESTADO Y DE CONTROL

% VECTOR DE ESTADO
x1=X(1);      % u
x2=X(2);      % v
x3=X(3);      % w
x4=X(4);      % p
x5=X(5);      % q
x6=X(6);      % r
x7=X(7);      % phi (Balanceo)
x8=X(8);      % theta (Cabeceo)
x9=X(9);      % phi (Guiñada)

% VECTOR DE CONTROL
u1=U(1);      %d_A (aileron)
u2=U(2);      %d_E (elevator)
u3=U(3);      %d_R (rudder)
u4=U(4);      %d_th1 (throttle 1)
u5=U(5);      %d_th2 (throttle 2)

% ------------------------- Variables  -------------------------

Va = sqrt(x1^2+x2^2+x3^2);
alpha = atan2(x3,x1);
beta=asin(x2/Va);
Q = 1/2*rho*Va^2;

% Velocidad y velocidad angular
V_b = [x1; x2; x3];
wbe_b = [x4; x5; x6];

%  Otros parámetos
rho          = rho_h;            % Densidad kg/m^3
g            = 9.81;             % Gravedad (m/s^2)


%% 3º Coeficientes fuerzas aerodinámicas

% CL_wb (wing + body)
if alpha <= alpha_switch
    CL_wb = n*(alpha - alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

% CL_t (tail)
epsilon = depsda*(alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;   % q (rad/s)
CL_t    = 3.1*(St/S)*alpha_t;

% CL total
CL = CL_wb + CL_t;     %EJES ESTABILIDAD

% CD 
CD = 0.13 + 0.07*(0.5*alpha + 0.654)^2;   %EJES ESTABILIDAD

% CY
CY = -1.6*beta + 0.24*u3;  % u3: rudder (rad)    %EJES ESTABILIDAD


%% 4º Fuerzas aerodinámicas

% F_A eje estabilidad
FA_s = [ -CD*Q*S;
          CY*Q*S;
         -CL*Q*S ];

% F_A eje cuerpo
C_bs = [ cos(alpha)  0  -sin(alpha);
         0           1   0;
         sin(alpha)  0   cos(alpha) ];

FA_b = C_bs * FA_s;    %FUERZAS AERODINÁMICAS EN EJES CUERPO

%% 5º Coeficiente momentos aerodinámicos AC

% Momentos aerodinámicos en el AC

coef_1 = -1.4*beta;
coef_2 = -0.59 - (3.1*(St*lt)/(S*mac))*(alpha - epsilon);
coef_3 = (1 - alpha*(180/(15*pi)))*beta;

coef_123 = [coef_1;
       coef_2;
       coef_3];

coef222 = (mac/Va) * [ -11      0                             5;
                       0  (-4.03*(St*lt^2)/(S*mac^2))         0;
                      1.7      0                           -11.5 ];

coef333 = [ -0.6    0                       0.22;
           0   (-3.1*(St*lt)/(S*mac))     0;
           0      0                      -0.63 ];

% Lo pasamos a ejes cuerpo
C_Mac_b = coef_123 + coef222 * wbe_b + coef333 * [u1; u2; u3];    


%% 6º Momentos aerodinámicos AC

% Dimensionalizan momentos aerodinámicos en AC

MAac_b=C_Mac_b*Q*S*mac; %Este modelo usa mac para todos los momentos
 

%% 7º Momentos aerodinámicos CG

%Momentos de AC a CG
rcg_b =[Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];

MAcg_b=MAac_b+cross(FA_b,rcg_b-rac_b); % Momentos aerodinámicos + Momentos generados por las fuerzas aerodinámicas

%% 8º Motor

% Empujes
F1=u4*m*g;
F2=u5*m*g;

%Vectorizado
FE1_b=[F1;0;0];
FE2_b=[F2;0;0];

FE_b=FE1_b+FE2_b;

%Momento empuje respecto CG
mew1=[Xcg-Xapt1;Yapt1-Ycg;Zcg-Zapt1];
mew2=[Xcg-Xapt2;Yapt2-Ycg;Zcg-Zapt2];

MEcg1_b=cross(mew1,FE1_b);
MEcg2_b=cross(mew2,FE2_b);

MEcg_b=MEcg1_b+MEcg2_b;


%% 9º Gravedad
% CFuerza gravitatoria en ejes cuerpo
g_b=[-g*sin(x8);g*cos(x8)*sin(x7);g*cos(x8)*cos(x7)];

Fg_b=m*g_b;  %Aplicado en CG


%% 10º OUTPUT 
% F Aero + Motor + Gravedad

F_b=Fg_b+FE_b+FA_b;

x1to3dot=(1/m)*F_b-cross(wbe_b,V_b);

% M Aero + Motor

Mcg_b=MAcg_b+MEcg_b;

x4to6dot=invIb*(Mcg_b-cross(wbe_b,Ib*wbe_b));

% Cinemática

H=[1 sin(x7)*tan(x8) cos(x7)*tan(x8);0 cos(x7) -sin(x7);0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9dot=H*wbe_b;

% Velocidad eje horizonte local
C1h=[cos(x9) sin(x9) 0;
    -sin(x9) cos(x9) 0;
    0 0 1];
C21=[cos(x8) 0 -sin(x8);
    0 1 0;
    sin(x8) 0 cos(x8)];
Cb2_fix=[1 0 0;
    0 sin(x7) cos(x7);
    0 -sin(x7) cos(x7)];

Cbh=Cb2_fix*C21*C1v;
Chb=Cbh';

x10to12dot=Chb*V_b;

%% 11º SALIDA ----------------------------
XDOT=[x1to3dot;
    x4to6dot;
    x7to9dot;
    x10to12dot];
















%%

%%