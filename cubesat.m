function dstate_dt = cubesat(t, state)
    %%state_0 = [x_0; y_0; z_0; vel_x_0; vel_y_0; vel_z_0];
    global BX BY BZ

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
    rho = norm(r);
    r_hat = r / rho;
    F_grav = -(grav_const*m_earth*m_cubesat/rho^2) *r_hat;


    %% X, Y, Z into Lat, Lon, Alt
    phi_E = 0;
    theta_E = acos(state(3) / rho);
    psi_E = atan2(state(2), state(1));

    latitude = 90 - theta_E * 180 / pi;
    longitude = psi_E * 180 / pi;
    altitude = (rho)/1000; %from center of earth in km
    

    %% Call mag field
    [BXN, BYE, BZD] = igrf('01-jan-2020', latitude, longitude, altitude, 'geocentric');


    %% North, east, down, to X, Y, Z
    B_NED = [BXN; BYE; -BZD]; %down is up
    BI = inertial_transformation_matrix(phi_E, theta_E + pi, psi_E) * B_NED;

    BX = BI(1);
    BY = BI(2);
    BZ = BI(3);

    %% dynamics
    F = F_grav;
    accel = F/m_cubesat;


    %% return derivatives vector
    dstate_dt = [vel; accel];



end