clear all; clc; close all;format long;
%% Definition of parameters
% we construct the structure that can be given as an argument to a block To Workspace
%T=5;                           % simulation time 
Ts = 0.1;
%timp=0:Ts:T;

%% PID constants
Kp = 1.2; %Proportinnal gain
Ki = 0.47; %Integral gain
Kd = 0; %Derivative gain

%% Turtlebot characteristics
r = 33e-3; %Wheel radius
d = 160e-3; %Distance between wheels
A = [r/2 r/2; r/d -r/d]; %[V;w]=A*[wr;wl]
%% Reference for path tracking
%reference end point for constant and linear references
%xref=-7;
%yref=26;
x0=[0 0 0];

%constant reference
%position_ref = [xref*ones(1,length(timp)); yref*ones(1,length(timp))];

%origin
%position_ref = [zeros(1,length(timp)); zeros(1,length(timp))];

%Linear reference
%position_ref = [xref*timp/T+5; yref*timp/T-2];

%Quadratic reference
%position_ref = [xref*timp.^2/T^2+5-1/4*timp/T; yref*timp.^2/T^2+1/4*timp/T-2];

%Circular references
%R=0.5; f1=0.02; f2=0.05;
%position_ref=[R*sin(f1*timp)+R*sin(f2*timp); R*cos(f1*timp)+R*cos(f2*timp) ];
%position_ref=[(R*sin(f1*timp));(R*cos(f1*timp))];

% spline reference
% Define points that the spline will pass through
x = [0 0.5 0.25 -0.5 -0.7];
y = [0 0.5 1 0 -0.5];

% Time vector
t = [0 10 20 30 35]; %Specify time reaching each chackpoint
time = 0:Ts:max(t);
T=max(t);

% Computing spline
xspline = spline(t,x,time);
yspline = spline(t,y,time);
ref = [xspline;yspline];

%% turtlebot simulation
load_system('turtlebot');  % we load the simulink model into memory
set_param('turtlebot', 'StopTime', num2str(T)) % set the simulation time

rsim=timeseries(ref',time);   % we build the structure that is received by the From Workspace block
out=sim('turtlebot');  % we run the simulink model, at the end of the simulation we have stored the output in the ysim structure
%% plot the results    
figure; grid on; hold on
plot(out.ysim.time,out.ysim.signals.values, 'm')
plot(rsim.Time,rsim.Data,'b')
legend('output - x ','output - y ','reference - xr','reference - yr')
xlabel('time')
ylabel('output signals')

figure; grid on; hold on
plot(out.ysim.signals.values(:,1) ,out.ysim.signals.values(:,2), 'om')
plot(rsim.Data(:,1),rsim.Data(:,2),'b')
legend('output - (x;y) ','reference - (xr;yr)')
xlabel('time')
ylabel('output signals')

xsim = out.ysim.signals.values(:,1)';
ysim = out.ysim.signals.values(:,2)';

error = sqrt((xsim(1:length(xspline))-xspline).^2 + ((ysim(1:length(xspline))-yspline).^2));
avg_error = mean(error)
