function [FVAL] = RCAM_model_implicito(XDOT, X, U);

FVAL = RCAM_model_pdg_short(X, U) - XDOT;

end