 % VUelo recto y a nivel
Xdot0 = [0;0;0;0;0;0;0;0;0];

X0 = [84.9905;0;1.2713;0;0;0;0;0.0150;0];

U0 = [0;-0.1780;0;0;0;0;0;0.0821;0.0821];

%Matrices aux
dxdot_matrix = 10e-12*ones(9,9);
dx_matrix    = 10e-12*ones(9,9);
du_matrix    = 10e-12*ones(9,5);

[E, Ap, Bp] = linealizacion_aux(@RCAM_model_implicito, Xdot0, X0, U0, ...
                             dxdot_matrix, dx_matrix, du_matrix);

%A & B
A = -inv(E)*Ap;
B = -inv(E)*Bp;

A
B
E

[A_lm, B_lm, C_lm, D_lm] = linmod('RCAM_model_sim_v5_LIN', X0, U0);

max(max(abs(B_lm-B)))
max(max(abs(A_lm-A)))

%%
%Guardar .mat 
save('matrices_A_B_trim44.mat', 'A', 'B');