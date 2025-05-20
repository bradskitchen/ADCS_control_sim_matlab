function [BfieldNav, pqrNav] = Navigation(BfieldMeasured, pqrMeasured)
    global BfieldNavprev pqrNavprev
    
    s = 0.3;

    if sum(BfieldNavprev) + sum(pqrNavprev) == 0
        BfieldNav = BfieldMeasured;
        pqrNav = pqrMeasured;
    else
        bias_estimate = [0; 0; 0];
        BfieldNav = BfieldNavprev*(1 - s) + s*(BfieldMeasured - bias_estimate);
        pqr_bias_estimate = [0; 0; 0];
        pqrNav = pqrNavprev*(1 - s) + s*(pqrMeasured - pqr_bias_estimate);
    end

    BfieldNavprev = BfieldNav;
    pqrNavprev = pqrNav;

end