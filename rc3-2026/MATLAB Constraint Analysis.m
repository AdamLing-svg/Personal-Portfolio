clear; close all; clc;

FT2M = 0.3048;
M2FT = 1/FT2M;

LB2N = 4.448222;
N2LB = 1/LB2N;

WS2IMPERIAL = N2LB/(M2FT^2);   % N/m^2 to lb/ft^2, for command-window summary only
PW2IMPERIAL = 1/N2LB;          % W/N to W/lb, for command-window summary only

WS2METRIC = LB2N/(FT2M^2);   % lb/ft^2 to N/m^2
PW2METRIC = 1/LB2N;          % W/lb to W/N

g = 9.81;
rho_SL = 1.225;
rho_CR = 1.225;
year = 2026;

% Speeds:
% V_S_target = target stall speed used for the stall W/S limit
% V_CR = representative delivery cruise / transit speed
% V_BK = representative banked-turn speed, set to 85% of cruise
% V_CL = representative climb speed
% V_RC = required rate of climb

% Propulsion:
% prop_eff = overall propulsive efficiency, using a conservative real-world value

% Aerodynamics:
% AR = aspect ratio, influences induced drag
% CD0  = parasite drag coefficient, estimates airframe cleanliness
% CLMAX_C = clean CLmax, used for stall limit
% CLMAX_T = takeoff/config CLmax, used in takeoff roll model

% Mission constraints:
% dist_TO = required liftoff distance
% bank_max = maximum bank angle for maneuvering constraint
% runway_mat = runway surface model, sets rolling friction

% V_S_target is calculated below from MTOW, wing area, and CLmax.

V_CR = 13.0;
V_BK = 0.85*V_CR;
V_CL = 10.7;
V_RC = 3.05;

prop_eff = 0.63;
CD0      = 0.045;
AR       = 7.0;

% Revised CDR geometry
b_ref = 1.50;             % wingspan [m]
S_ref = b_ref^2/AR;       % wing reference area [m^2]
c_ref = S_ref/b_ref;     % mean chord estimate [m]

% Revised CDR mission constraints
dist_TO  = 10.7;          % 35 ft converted to metres, rounded
bank_max = 45;

% Conservative lift assumptions for CDR
CLMAX_C  = 1.50;
CLMAX_T  = 1.60;

runway_mat = 'soft_ground';

% Mass & power caps based on the provided RC3 power system

m_takeoff_kg = 2.41000;    % Updated MTOW / speed-case mass [kg]
W_takeoff_N  = m_takeoff_kg * g;

% Calculated stall speed at MTOW using unchanged geometry and clean CLmax.
V_S_target = sqrt((2*W_takeoff_N)/(rho_CR*S_ref*CLMAX_C));

% Provided system: 3S 2200 mAh LiPo + 40 A ESC
V_batt_nominal = 11.1;    % nominal 3S LiPo voltage [V]
V_batt_full    = 12.6;    % fully charged 3S LiPo voltage [V]
I_ESC_max      = 40;      % ESC current rating [A]

P_nominal_W = V_batt_nominal * I_ESC_max;  % nominal electrical cap [W]
P_peak_W    = V_batt_full    * I_ESC_max;  % fresh-battery upper-bound estimate [W]
P_cont_W    = 0.80 * P_nominal_W;          % conservative continuous cap [W]

P_cap_W   = P_nominal_W;
P_ideal_W = P_cont_W;

PW_cap_SI   = P_cap_W   / W_takeoff_N;
PW_ideal_SI = P_ideal_W / W_takeoff_N;

% Plot limits [metric/SI]

min_wing_loading = 20;    % [N/m^2]
max_wing_loading = 120;   % [N/m^2]

min_power_to_wgt = 0;     % [W/N]
max_power_to_wgt = 28;    % [W/N]

% Derived aero properties

e  = 1.78*(1 - 0.045*(AR^0.68)) - 0.64;
k  = 1/(pi*e*AR);

CLTO = CLMAX_T/1.21;
mu_g = runway_friction(runway_mat);

% Constraint curves [metric/SI]

[PW_TO, WS_TO] = takeoff_curve(min_wing_loading, max_wing_loading, rho_SL, g, CLTO, dist_TO, mu_g, CD0, prop_eff);
[PW_CR, WS_CR] = banked_cruise(0.0,      min_wing_loading, max_wing_loading, rho_CR, V_CR, prop_eff, CD0, k);
[PW_BK, WS_BK] = banked_cruise(bank_max, min_wing_loading, max_wing_loading, rho_CR, V_BK, prop_eff, CD0, k);
[PW_CL, WS_CL] = climb_curve(min_wing_loading, max_wing_loading, rho_CR, V_CL, V_RC, prop_eff, CD0, k);

WS_stall = stall_wing_loading(rho_CR, V_S_target, CLMAX_C);

[PW_CRo, WS_CRo] = banked_cruise_optimal(0.0,      min_wing_loading, max_wing_loading, rho_CR, V_S_target, prop_eff, CD0, k, 'endurance');
[PW_BKo, WS_BKo] = banked_cruise_optimal(bank_max, min_wing_loading, max_wing_loading, rho_CR, V_S_target, prop_eff, CD0, k, 'endurance');
[PW_CLo, WS_CLo] = climb_optimal(min_wing_loading, max_wing_loading, rho_CR, V_S_target, V_RC, prop_eff, CD0, k);

% Current design point

WS_design = W_takeoff_N/S_ref;
PW_design_available = PW_ideal_SI;

PW_TO_design = interp1(WS_TO, PW_TO, WS_design, 'linear', 'extrap');
PW_CR_design = interp1(WS_CR, PW_CR, WS_design, 'linear', 'extrap');
PW_BK_design = interp1(WS_BK, PW_BK, WS_design, 'linear', 'extrap');
PW_CL_design = interp1(WS_CL, PW_CL, WS_design, 'linear', 'extrap');

P_TO_design = PW_TO_design * W_takeoff_N;
P_CR_design = PW_CR_design * W_takeoff_N;
P_BK_design = PW_BK_design * W_takeoff_N;
P_CL_design = PW_CL_design * W_takeoff_N;

% Stall estimates at selected geometry

V_stall_CL15 = sqrt((2*W_takeoff_N)/(rho_CR*S_ref*1.50));
V_stall_CL16 = sqrt((2*W_takeoff_N)/(rho_CR*S_ref*1.60));

n_bank = 1/cosd(bank_max);
V_bank_stall_CL15 = V_stall_CL15*sqrt(n_bank);
bank_speed_margin = V_BK/V_bank_stall_CL15;

xmax = max_wing_loading;
ymin = min_power_to_wgt;
ymax = max_power_to_wgt;

% Plots

figure('Color','w');
title(sprintf('Delivery Stream Constraint Analysis (%d) | m=%.2f kg | b=%.2f m | AR=%.1f', year, m_takeoff_kg, b_ref, AR));
xlabel('Wing Loading W/S [N/m^2]');
ylabel('Power-to-Weight P/W [W/N]');
xlim([0, xmax]);
ylim([ymin, ymax]);
grid on; hold on;

plot(WS_TO, PW_TO, 'r', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Takeoff (s_{TO}=%.1f m)', dist_TO));

plot(WS_CR, PW_CR, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Level Cruise (V=%.1f m/s)', V_CR));

plot(WS_BK, PW_BK, 'y', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Banked Cruise (\phi=%.0f^\circ, V=%.1f m/s)', bank_max, V_BK));

plot(WS_CL, PW_CL, 'g', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Climb (V=%.1f m/s, ROC=%.2f m/s)', V_CL, V_RC));

plot(WS_CRo, PW_CRo, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.3, ...
    'DisplayName', 'Level Cruise (optimal V, endurance)');

plot(WS_BKo, PW_BKo, 'y--', 'LineWidth', 1.3, ...
    'DisplayName', sprintf('Banked Cruise (optimal V, endurance, \phi=%.0f^\circ)', bank_max));

plot(WS_CLo, PW_CLo, 'g--', 'LineWidth', 1.3, ...
    'DisplayName', 'Climb (optimal V, endurance)');

xline(WS_stall, 'b', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Stall at MTOW (V_S=%.2f m/s)', V_S_target));

yline(PW_ideal_SI, 'k--', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Conservative Continuous Power Cap (%.0f W)', P_ideal_W));

yline(PW_cap_SI, 'k-', 'LineWidth', 1.6, ...
    'DisplayName', sprintf('Nominal Electrical Power Cap (%.0f W)', P_cap_W));

legend('Location', 'best');
hold off;

% Command-window summary

fprintf('\n===== RC3 Delivery CDR Metric Constraint Analysis Summary =====\n');
fprintf('MTOW / speed-case mass used in analysis: %.5f kg\n', m_takeoff_kg);
fprintf('Wingspan:                           %.2f m\n', b_ref);
fprintf('Aspect ratio:                       %.2f\n', AR);
fprintf('Wing area:                          %.3f m^2\n', S_ref);
fprintf('Mean chord estimate:                %.3f m\n', c_ref);
fprintf('Design wing loading:                %.1f N/m^2\n', WS_design);
fprintf('Design wing loading:                %.2f lb/ft^2\n', WS_design*WS2IMPERIAL);
fprintf('Cruise speed:                       %.2f m/s\n', V_CR);
fprintf('Banked-turn speed, 0.85*V_CR:       %.2f m/s\n', V_BK);
fprintf('Climb speed:                        %.2f m/s\n', V_CL);
fprintf('Required ROC:                       %.2f m/s\n', V_RC);
fprintf('Bank angle:                         %.0f deg\n', bank_max);
fprintf('Clean CLmax used for stall line:    %.2f\n', CLMAX_C);
fprintf('Takeoff CLmax used in TO model:     %.2f\n', CLMAX_T);
fprintf('Calculated stall speed at MTOW, CLmax=1.50: %.2f m/s\n', V_stall_CL15);
fprintf('Calculated stall speed at MTOW, CLmax=1.60: %.2f m/s\n', V_stall_CL16);
fprintf('45-deg bank stall, CLmax=1.50:      %.2f m/s\n', V_bank_stall_CL15);
fprintf('Bank speed / bank-stall margin:     %.2fx\n', bank_speed_margin);
fprintf('\nProvided power system estimate:\n');
fprintf('Conservative continuous power:      %.0f W\n', P_cont_W);
fprintf('Nominal electrical power cap:       %.0f W\n', P_nominal_W);
fprintf('Fresh-battery upper-bound estimate: %.0f W\n', P_peak_W);
fprintf('Takeoff:                            %.1f W\n', P_TO_design);
fprintf('Level cruise:                       %.1f W\n', P_CR_design);
fprintf('Banked cruise:                      %.1f W\n', P_BK_design);
fprintf('Climb:                              %.1f W\n', P_CL_design);
fprintf('Limiting required power:            %.1f W\n', max([P_TO_design, P_CR_design, P_BK_design, P_CL_design]));
fprintf('Power margin vs continuous cap:     %.2fx\n', P_cont_W/max([P_TO_design, P_CR_design, P_BK_design, P_CL_design]));
fprintf('===============================================================\n\n');

% Local functions

function mu = runway_friction(material)
    switch material
        case 'concrete'
            mu = 0.025;
        case 'soft_ground'
            mu = 0.20;
        otherwise
            error('Unknown runway material: %s', material);
    end
end

function [PW, WS] = takeoff_curve(minWS, maxWS, rho_SL, g, CLTO, dist_TO, mu_g, CD0, prop_eff)
    WS = linspace(minWS, maxWS, 100);
    distance_factor = WS./(rho_SL*g*CLTO*dist_TO);
    friction_factor = mu_g;
    parasite_factor = CD0/(2*CLTO);
    PW_coeff = prop_eff .* sqrt(rho_SL*CLTO./WS);
    PW = (1./PW_coeff) .* (distance_factor + friction_factor + parasite_factor);
end

function [PW, WS] = banked_cruise(bank_angle_deg, minWS, maxWS, rho_CR, V, prop_eff, CD0, k)
    WS = linspace(minWS, maxWS, 100);
    q  = 0.5*rho_CR*V^2;
    n  = 1/cosd(bank_angle_deg);
    PW = (V/prop_eff) .* ( q.*CD0.*(1./WS) + (k*n^2).*(WS./q) );
end

function [PW, WS] = climb_curve(minWS, maxWS, rho_CR, V_CL, V_RC, prop_eff, CD0, k)
    WS = linspace(minWS, maxWS, 100);
    q  = 0.5*rho_CR*V_CL^2;
    PW = (V_RC/prop_eff) + (V_CL/prop_eff) .* ( q.*CD0.*(1./WS) + k.*(WS./q) );
end

function WS_stall = stall_wing_loading(rho_CR, V_S, CLMAX_C)
    q_stall  = 0.5*rho_CR*V_S^2;
    WS_stall = q_stall*CLMAX_C;
end

function [PW, WS] = banked_cruise_optimal(bank_angle_deg, minWS, maxWS, rho_CR, V_S, prop_eff, CD0, k, best_mode)
    WS = linspace(minWS, maxWS, 100);
    switch best_mode
        case 'endurance'
            V_best = (4*k*(WS.^2)./(3*(rho_CR^2)*CD0)).^(1/4);
        case 'range'
            V_best = (4*k*(WS.^2)./((rho_CR^2)*CD0)).^(1/4);
        otherwise
            error('best_mode must be ''endurance'' or ''range''');
    end
    V_best = max(V_best, 1.1*V_S);
    q      = 0.5*rho_CR*(V_best.^2);
    n      = 1/cosd(bank_angle_deg);
    PW = (V_best/prop_eff) .* ( q.*CD0.*(1./WS) + (k*n^2).*(WS./q) );
end

function [PW, WS] = climb_optimal(minWS, maxWS, rho_CR, V_S, V_RC, prop_eff, CD0, k)
    WS = linspace(minWS, maxWS, 100);
    V_best = (4*k*(WS.^2)./(3*(rho_CR^2)*CD0)).^(1/4);
    V_best = max(V_best, 1.1*V_S);
    q = 0.5*rho_CR*(V_best.^2);
    PW = (V_RC/prop_eff) + (V_best/prop_eff) .* ( q.*CD0.*(1./WS) + k.*(WS./q) );
end
