%Adam Ling
%501287341
%Mechanisms and Vibrations AER403
%Section 7

close all;
clear;
clc;
ratio=pi/180; %Convert degrees to radians, then set the CB driver and BA coupler lengths to 15mm and 45mm
r2=25;
r3=75; 
rpmOfLink=95; %Define angular velocity
rpmOfLinkinRadians=rpmOfLink*2*pi/60; 

One_cycle=2*pi/rpmOfLinkinRadians; %Set the time for one full rotation of the crank, then do two rotations, and set the time step
TwoTimesTotal=2*One_cycle;
Increment=0.001;
t=0:Increment:TwoTimesTotal;

figure %Open a figure
axis([-5 65 -20 20]) %Set the axis so the whole mechanism fits on the graph
axis equal
axis manual
grid on
title('Slider-Crank Animation') %Add labels
xlabel('x (mm)')
ylabel('y (mm)')

%Create empty plot objects so animation is smoother and faster (there were
%issues with the animation without this part, so I added it)
hold on
h1=plot([0 0],[0 0],'o-','LineWidth',2); %CB
h2=plot([0 0],[0 0],'o-','LineWidth',2); %BA
h3=plot([0 65],[0 0],'k--'); %Pathway for slider
hold off

for i=1:length(t) %Here, most of the program loops to make a bunch of plots and animate as a result
    
    theta_2(i)=rpmOfLinkinRadians*t(i); %Instead of prescribing slider motion like the old lab, the crank angle is now the known input
    
    %Start position analysis by finding x and y positions for crank
    xB=r2*cos(theta_2(i)); %x-position
    yB=r2*sin(theta_2(i)); %y-position 
    
    r1(i)=xB + sqrt(r3^2 - yB^2); %This is now the slider position output from the layout/geometry 
    
    %Start analysis of points, where A is the slider joint, B is the moving joint, and C is the fixed crank pivot
    zz(1,:)=[r1(i), 0]; %Goes in order of point A (this line), B, C, then back to A
    zz(2,:)=[xB, yB]; 
    zz(3,:)=[0, 0];
    zz(4,:)=zz(1,:); 

    theta_3(i)=atan2(zz(2,2)-zz(1,2), zz(2,1)-zz(1,1)); %For the angle of coupler AB, we can use atan2 function in MATLAB, which makes things easier
    
    set(h1,'XData',[zz(3,1) zz(2,1)],'YData',[zz(3,2) zz(2,2)]) %Animate the driving crank CB (this line) then BA
    set(h2,'XData',[zz(2,1) zz(1,1)],'YData',[zz(2,2) zz(1,2)]) 
    drawnow limitrate
    pause(0.001);
end

%Start plotting the velocity of the actual slider
v_slider=gradient(r1, t); %Find slope of slider displacement via gradient function

figure %Open new figure for graph then plot the slider velocity, which is important determine stride length and performance later on
plot(t, v_slider)
grid on
title('Slider Velocity vs Time (2 Rotations)') %Add title and label for axis
xlabel('Time (s)')
ylabel('Velocity (mm/s)')