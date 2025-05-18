%%Fourth Order Runge-Kutta ODE integrator
%%Author: Brayden Altman
%
% Inputs:
%       odefun: Function handle for the ODE system
%       tspan:  Time span [t_start, t_end]
%       y0:     Initial state vector
%       h:      Step size
%
% Outputs:
%       t_out: Vector of time points
%       y_out: Matrix of state vectors at each time point
function [t_out, y_out] = rk4(odefun, tspan, y0, h)
    t_start = tspan(1);
    t_end = tspan(2);


    % Determine the number of steps
    num_steps = ceil((t-end - t_start) / h);

    %adjust step size to match time exactly
    h = (t_end - t_start) / n_steps;

    %Initalize output
    t_out = zeros(n_steps + 1, 1);
    y_out = zeros(n_steps + 1, length(y0));

    %% Integration loop
    for i = 1:n_steps
        t = t_out(i);
        y = y_out(i, :)';

        %calculate RK4 coefficients
        k1 = odefun(t, y);
        k2 = odefun(t + h/2, y + h*(k1)/2);
        k3 = odefun(t + h/2, y + h*(k2)/2);
        k4 = odefun(t + h, y + h*(k3));

        %update time and state
        t_out(i+1) = t+h;
        y_out(i+1, :) = y' + h/6 * (k1 + 2*(k2) + 2*(k3) +k4)';
    end
end


