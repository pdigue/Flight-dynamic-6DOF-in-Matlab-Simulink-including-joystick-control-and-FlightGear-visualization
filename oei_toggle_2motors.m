function [engines_output] = oei_toggle_2motors(motor1_in, motor2_in, cmd)
% Conmuta cada motor entre su entrada y 0 según 'cmd'

% --- Parámetros (ajusta a tus valores) ---
A   = 270;   % pulso que conmuta motor 1
B   = 90;    % pulso que conmuta motor 2
tol = 0.5;   % tolerancia
-
persistent off1 off2 prevA prevB initialized
if isempty(initialized) || ~initialized
% Arrancar con motores ON (pasando la señal)
    off1 = false;
    off2 = false;
    prevA = false;
    prevB = false;
    initialized = true;
end

isA = abs(cmd - A) < tol;
isB = abs(cmd - B) < tol;

pulseA = isA && ~prevA;
pulseB = isB && ~prevB;
prevA  = isA;
prevB  = isB;

if pulseA, off1 = ~off1; end
if pulseB, off2 = ~off2; end

% --- Salidas ---
motor1_out = motor1_in;
motor2_out = motor2_in;
if off1, motor1_out = 0; end
if off2, motor2_out = 0; end

engines_output = [motor1_out;motor2_out];

end
