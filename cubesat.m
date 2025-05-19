function dstate_dt = cubesat(t, state)
    %%state_0 = [x_0; y_0; z_0; vel_x_0; vel_y_0; vel_z_0];
    global BB inv_inertia inertia m nextMagUpdate lastMagUpdate
    global lastSensorUpdate nextSensorUpdate BfieldMeasured pqrMeasured

    %x = state(1);
    %y = state(2);
    %z = state(3);


    %vel_x = state(4);
    %vel_y = state(5);
    %vel_z = state(6);

    
    quanternions = state(7:10);
    p = state(11);
    q = state(12);
    r = state(13);
    pqr = state(11:13);

    

    %% inertia and mass
    m_cubesat = 4;

    inertia = [.04187, 0, 0; 0, .04187, 0; 0, 0, .00667];
    inv_inertia = inv(inertia);

    inertia_dot = 0; % for now zero, will adjust for antenna and boom arm later


    %% translational kinematics
    vel = state(4:6);

    %% rotational kinematics

    PQRMAT = [0, -p, -q, -r; p, 0, r, -q; q, -r, 0, p; r, q, -p, 0];
    quant_dot = .5*PQRMAT*quanternions;
    

    %% gravity model
    [radius, m_earth, grav_const, mu] = earth();
    r = state(1:3);
    rho = norm(r);
    r_hat = r / rho;
    F_grav = -(grav_const*m_earth*m_cubesat/rho^2) *r_hat;

    if t >= lastMagUpdate
        lastMagUpdate = lastMagUpdate + nextMagUpdate;
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
    

        BB = TIBquat(quanternions)'*BI;

    end

    if t >= lastSensorUpdate
        lastSensorUpdate = lastSensorUpdate + nextSensorUpdate;
        [BfieldMeasured, pqrMeasured] = Sensor(BB, pqr);
    end
    %% translational dynamics
    F = F_grav;
    accel = F/m_cubesat;

    %% magtourquer model
    mangnetorquers = [0;0;0];

    %% rotational dynamics
    angular_momentum = inertia * pqr;
    pqr_dot = inv_inertia*(mangnetorquers - cross(pqr, angular_momentum) - inertia_dot);


    %% return derivatives vector
    dstate_dt = [vel; accel; quant_dot; pqr_dot];



end