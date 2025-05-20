function current = Control(BfieldNav, pqrNav)

    k = 67200;
    %%Bfield is in tesla - 40000 nT = 4e-5 T ~= 1e-5 T
    %%pqr is in rad/s --   0.1 rad/s = 1e-1
    %%pqr*bfield = 1e-6
    %%(pqr*bfield)/(n*A) = 6e-7
    %%muB = n*i*A
    %%guess
    [n, A] = magtorquer_params();
    current = k * cross(pqrNav, BfieldNav);
    %%current to be in amps ~= 40mA = 4e-2A

    %%add in saturation
    if sum(abs(current)) > .04
        current = current/norm(current)*.04;
    end
end