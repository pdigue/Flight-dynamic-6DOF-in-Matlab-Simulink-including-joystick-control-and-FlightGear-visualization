clear
close all

%Iz_guess
initialization = 0;         %0 = guess random
                            %1 = usas el mat
if (initialization==1)
    z_guess = zeros(14,1);
    z_guess(1) = 85;
else
    load trim_values_straight_level
    z_guess = [XStar;UStar];
end

[ZStar,f0] = fminsearch('trimming_aux', z_guess, ...
    optimset('TolX',1e-10,'MaxFunEvals',10000,'MaxIter',10000))

XStar = ZStar(1:9);
UStar = ZStar(10:14);

%verificación
XdotStar   = RCAM_model_pdg_short(XStar,UStar);
VaStar     = sqrt(XStar(1)^2 + XStar(2)^2 + XStar(3)^2);
gammaStar  = XStar(8) - atan2(XStar(3),XStar(1));
vStar      = XStar(2);
phiStar    = XStar(7);
psiStar    = XStar(9);

save trim_values_straight_level XStar UStar
