function [P_dot, Q_dot, R_dot] = MomentEquations(moments_A, moments_T, rates, constants)
    % Inputs:
    % moments_A = [lA; mA; nA] (Aerodynamic moments)
    % moments_T = [lT; mT; nT] (Engine moments)
    % rates     = [P; Q; R]    (Body rates)
    % constants = [Jx; Jy; Jz; Jxz; Gamma]

    % Unpack inputs
    lA = moments_A(1); mA = moments_A(2); nA = moments_A(3);
    lT = moments_T(1); mT = moments_T(2); nT = moments_T(3);
    P = rates(1); Q = rates(2); R = rates(3);
    
    Jx = constants(1); Jy = constants(2); Jz = constants(3); 
    Jxz = constants(4); Gamma = constants(5);

    % Moment Equations [cite: 689, 690]
    P_dot = (1/Gamma) * (Jxz*(Jx - Jy + Jz)*P*Q - (Jz*(Jz - Jy) + Jxz^2)*Q*R + Jz*(lA + lT) + Jxz*(nA + nT));
    Q_dot = (1/Jy) * ((Jz - Jx)*P*R - Jxz*(P^2 - R^2) + mA + mT);
    R_dot = (1/Gamma) * (((Jx - Jy)*Jx + Jxz^2)*P*Q - Jxz*(Jx - Jy + Jz)*Q*R + Jxz*(lA + lT) + Jx*(nA + nT));
end