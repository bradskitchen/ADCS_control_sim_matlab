function dstate_dt = cubesat(t, state)
    %%state_0 = [x_0; y_0; z_0; vel_x_0; vel_y_0; vel_z_0];


    %x = state(1);
    %y = state(2);
    %z = state(3);


    %vel_x = state(4);
    %vel_y = state(5);
    %vel_z = state(6);


    %% inertia and mass
    m_cubesat = 4;


    %% kinematics
    vel = state(4:6);

    
    %% gravity model
    [radius, m_earth, grav_const, mu] = earth();
    r = state(1:3);
    r_norm = norm(r);
    r_hat = r / r_norm;
    F_grav = -(grav_const*m_earth*m_cubesat/r_norm^2) *r_hat;

    %% dynamics
    F = F_grav;
    accel = F/m_cubesat;


    %% return derivatives vector
    dstate_dt = [vel; accel];



end