%Ionela Prodan
clear all; clc; close all

% we construct the structure that can be given as an argument to a block To Workspace
T=200;                           % simulation time 
timp=linspace(0,T,1e4);         % simulation interval of 15 seconds
% r=sin(timp);                    % list of entries for the system
R=15; f1=0.02; f2=0.12;
r=[R*sin(f1*timp)+R*sin(f2*timp); R*cos(f1*timp)+R*cos(f2*timp) ];



%% pendulum system simuation
load_system('CarSim');  % we load the simulink model into memory
set_param('CarSim', 'StopTime', num2str(T)) % set the simulation time

rsim=timeseries(r',timp);   % we build the structure that is received by the From Workspace block
out=sim('CarSim')  % we run the simulink model, at the end of the simulation we have stored the output in the ysim structure
%% plot the results    
figure; grid on; hold on
plot(out.ysim.time,out.ysim.signals.values, 'm')
plot(rsim.Time,rsim.Data,'b')
legend('output - y1 ','output - y2 ','reference - r1','reference - r2')
xlabel('time')
ylabel('output signals')
