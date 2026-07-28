clear
clc
close all

%% initial conditions
%states
longitude0 = deg2rad(-79.625335);
latitude0 = deg2rad(43.676856);
altitude0 = 7224;
phi0 = 0;
theta0 = 0.1;
psi0 = 0;
U0 = 84.9905;
V0 = 0;
W0 = 1.2713;
P0 = 0;
Q0 = 0.0149;
R0 = 0;
X0 = [longitude0; latitude0; altitude0; phi0; theta0; psi0; U0; V0; W0; P0; Q0; R0];

% inputs
deltaT10 = 0.0821;
deltaT20 = 0.0821;
deltaA0 = 0;
deltaE0 = -0.1780;
deltaR0 = 0;
Ucmd0 = [deltaT10, deltaT20, deltaA0, deltaE0, deltaR0]';

%simulation time
TF = 50;
maxAileronDeflection = 25*(pi/180);
maxElevatorDeflection = 10*(pi/180);
maxRudderDeflection = 30*(pi/180);

m = 120000;
g = 9.81;

Jx  = m*40.07;
Jy  = m*64.00;
Jz  = m*99.92;
Jxz = -m*2.0923;
Gamma = Jx*Jz - Jxz^2;

forceConstants  = [m; g];
momentConstants = [Jx; Jy; Jz; Jxz; Gamma];

%% Run the model
out = sim("AircraftModelAdam.slx");

t = out.simX.Time;

x1 = out.simX.Data(:,1);
x2 = out.simX.Data(:,2);
x3 = out.simX.Data(:,3);
x4 = out.simX.Data(:,4);
x5 = out.simX.Data(:,5);
x6 = out.simX.Data(:,6);
x7 = out.simX.Data(:,7);
x8 = out.simX.Data(:,8);
x9 = out.simX.Data(:,9);
x10 = out.simX.Data(:,10);
x11 = out.simX.Data(:,11);
x12 = out.simX.Data(:,12);

figure 
subplot(4,3,1)
plot(t,x1)
legend('longitude')
grid on
subplot(4,3,2)
plot(t,x2)
legend('latitude')
grid on
subplot(4,3,3)
plot(t,x3)
legend('h')
grid on
subplot(4,3,4)
plot(t,x4)
legend('phi')
grid on
subplot(4,3,5)
plot(t,x5)
legend('theta')
grid on
subplot(4,3,6)
plot(t,x6)
legend('psi')
grid on
subplot(4,3,7)
plot(t,x7)
legend('U')
grid on
subplot(4,3,8)
plot(t,x8)
legend('V')
grid on
subplot(4,3,9)
plot(t,x9)
legend('W')
grid on
subplot(4,3,10)
plot(t,x10)
legend('P')
grid on
subplot(4,3,11)
plot(t,x11)
legend('Q')
grid on
subplot(4,3,12)
plot(t,x12)
legend('R')
grid on