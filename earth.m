%% Earth Parameters
function [radius, m_earth, grav_const, mu] = earth()
    radius = 6.371e6;
    m_earth = 5.972e24;
    grav_const = 6.67e-11;
    mu = m_earth * grav_const;
end
