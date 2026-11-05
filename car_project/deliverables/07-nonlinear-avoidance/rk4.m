function x_next = rk4(f, x, u, h)
    % Runge-Kutta 4th Order Integration
    %
    % Inputs:
    %   f  - Function handle representing the continuous dynamics, f(x, u)
    %   x  - Current state (vector)
    %   u  - Current input (vector)
    %   h  - Step size (sampling time)
    %
    % Output:
    %   x_next - State at the next timestep
    
    k1 = f(x, u);
    k2 = f(x + h/2 * k1, u);
    k3 = f(x + h/2 * k2, u);
    k4 = f(x + h * k3, u);
    x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end
