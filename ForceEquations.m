function [U_dot, V_dot, W_dot] = ForceEquations(forces_A, forces_T, rates, states, constants)
    % Inputs:
    % forces_A = [XA; YA; ZA] (Aerodynamic forces)
    % forces_T = [XT; YT; ZT] (Engine thrust forces)
    % rates    = [P; Q; R]    (Body rates)
    % states   = [U; V; W; phi; theta]
    % constants = [m; g]

    % Unpack inputs
    XA = forces_A(1); YA = forces_A(2); ZA = forces_A(3);
    XT = forces_T(1); YT = forces_T(2); ZT = forces_T(3);
    P = rates(1); Q = rates(2); R = rates(3);
    U = states(1); V = states(2); W = states(3);
    phi = states(4); theta = states(5);
    m = constants(1); g = constants(2);

    % Force Equations 
    U_dot = R*V - Q*W - g*sin(theta) + (XT + XA)/m;
    V_dot = -R*U + P*W + g*sin(phi)*cos(theta) + (YT + YA)/m;
    W_dot = Q*U - P*V + g*cos(phi)*cos(theta) + (ZT + ZA)/m;
end