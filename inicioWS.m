% Inicializar constantes  modelo RCAM
clear
clc
close all

%% 1º ESTADO INICIAL X & CONTROLES FIJOS & SATURACIONES
% SI SE DESEA, METER AQUÍ PUNTO DE TRIMADO
x0 = [84.990252;    % u: valor inicial
      0;            % v: velocidad en y
      1.2872895;    % w: velocidad en z 
      0;            % p: velocidad angular en x
      0;            % q: velocidad angular en y
      0;            % r: velocidad angular en z
      0;            % phi: ángulo de rol
    0.015145161;    % theta: ángulo de pitch
      0;            % psi: ángulo de yaw
      0;            % x: posición en x
      0;            % y: posición en y
     -4000];        % z: posición en z 

u = [ 0;               % A
     -0.1793987;       % E
      0;               % R
      0.078512;        % PROP1   
      0.078512];       % PROP2     

% Valores max & min saturaciones
u1min=-15*(pi/180);
u1max=25*(pi/180);

u2min=-25*(pi/180);
u2max=10*(pi/180);

u3min=-30*(pi/180);
u3max=30*(pi/180);

u4min= 0; %0.5*(pi/180);
u4max=10*(pi/180);

u5min=0*(pi/180);
u5max=10*(pi/180);

%% 2º DEFINICIÓN OTROS PARÁMETROS
Va0 = sqrt(x0(1)^2+x0(2)^2+x0(3)^2);

% Inicio FlightGear
lon0 = deg2rad(ConvertLatLonDegMinSecToDecimal(X,Y,Z,'W'));
lat0 = deg2rad(ConvertLatLonDegMinSecToDecimal(X,Y,Z,'N'));
h0 = XYZ;

Xgeodetic0 = [lat0; lon0; h0]; 

%% 3º DEFINIR TIEMPOS SIMULACIÓN
dt = 1/50; 
timeFinal = 720;

%% 4º DISEÑO DE MANIONBRAS ===========================

% ======================= OEI =======================
% Valor que anule el motor si queremos OEI, si no, mantener en 0
time_oei_forzado_1= 20000; %derecho
time_oei_forzado_2= 100;   %izquierdo
oei_forzado_value = 0;

% ======================= DOBLETES =======================
t_doblete_1_p1 = 50;
t_doblete_1_p2 = t_doblete_1_p1 + 2;
t_doblete_1_p3 = t_doblete_1_p2 + 1;
t_doblete_1_p4 = t_doblete_1_p3 + 2;
amplitud_doblete1 = 7 * pi / 180;

t_doblete_2_p1 = 250;
t_doblete_2_p2 = t_doblete_2_p1 + 3;
t_doblete_2_p3 = t_doblete_2_p2 + 1;
t_doblete_2_p4 = t_doblete_2_p3 + 3;
amplitud_doblete2 = 0 * pi / 180;

t_doblete_3_p1 = 200;
t_doblete_3_p2 = t_doblete_3_p1 + 3;
t_doblete_3_p3 = t_doblete_3_p2 + 1;
t_doblete_3_p4 = t_doblete_3_p3 + 3;
amplitud_doblete3 = 0 * pi / 180;

t_doblete_4_p1 = 200;
t_doblete_4_p2 = t_doblete_4_p1 + 3;
t_doblete_4_p3 = t_doblete_4_p2 + 1;
t_doblete_4_p4 = t_doblete_4_p3 + 3;
amplitud_doblete4 = 0 * pi / 180;

% ======================= IMPULSO / ESCALÓN =======================
% Se puede usar el impulso como escalón poniendo una duración del impulso
% que quede fuera del tiempo de ejecución, si no, usar escalón directamente

t_impulso_1 = 100;
duracion_impulso_1 = 6;
amplitud_impulso_1 = 10 * pi/180;

t_impulso_2 = 180;
duracion_impulso_2 = 5;
amplitud_impulso_2 = 7 * pi/180;

t_impulso_3 = 300;
duracion_impulso_3 = 4;
amplitud_impulso_3 = 0 * pi/180;

t_impulso_4 = 300;
duracion_impulso_4 = 4;
amplitud_impulso_4 = 0 * pi/180;

t_impulso_5 = 450;
duracion_impulso_5 = 400;
amplitud_impulso_5 = 4 * pi/180;

t_step1 = 250;
amplitud_step1 =  0 * pi/180;

t_step2 = 160;
amplitud_step2 = 10.9 * pi/180;

t_step3 = 260;
amplitud_step3 = 3 * pi/180;

t_step4 = 8000;
amplitud_step4 =  0 * pi/180;

t_step5 = 420;
amplitud_step5 =  0 * pi/180;

% ======================= INVERTIR DEFLEXIÓN ======================
% En t = t_inv_deflex_1_p1 se mete una deflexión, en t_inv_deflex_1_p2 se
% invierte la deflexión (e.g. de 5 a -5º). En t_back se da la opción de
% volver a cero (parecido al doblete pero separando las deflexiones)
t_inv_deflex_1_p1 = 100;
t_inv_deflex_1_p2 = t_inv_deflex_1_p1 + 120;
t_inv_deflex_1_p3 = t_inv_deflex_1_p2 + 120;
amplitud_ida_1 = 5 * pi/180;
amplitud_vuelta_1 = -amplitud_ida_1 * 2;

%% TIEMPOS MARCADORES GRÁFICA 3D
t_check_maniobra_1 = 0;
t_check_maniobra_2 = time_oei_forzado_2;
t_check_maniobra_3 = t_step2;
t_check_maniobra_4 = t_step3;
t_check_maniobra_5 = 0;
t_check_maniobra_6 = 0;
% t_check_maniobra_7 = 600;
% t_check_maniobra_8 = 850;


%% LANZAMOS EL MODELO
sim('RCAM_model_sim_rho')
out = sim('RCAM_model_sim_rho'); 
