clear all;% close all; clc;

% File with datapoints of the obstacle named xcarthography and ycarthography
load output_data.mat;

% Number of points:
N = length(xcarthography);

% xcarthography and ycarthography are row vectors
% We want to solve a matrix equation AX=B with:
A = [-2*xcarthography' -2*ycarthography' ones(N,1)];
B = -(xcarthography').^2 + -(ycarthography').^2;

% We compute A'AX=A'B, so X = inv(A'A)*A'B

X = inv(A'*A)*A'*B;

% Since X=[x0;y0;x0^2+y0^2-r^2], we retrieve x0, y0, and r
x0 = X(1)
y0 = X(2)
r=sqrt(x0^2 + y0^2 - X(3))

% Vectors for the computed circle
theta = 0:2*pi/360:2*pi;
x = (r*cos(theta)+x0);
y = (r*sin(theta)+y0);

% Exact values given in the simulation 
r_real = 0.139385;
x0_real = 0.889429;
y0_real = -0.044138;

x_real = (r_real*cos(theta)+x0_real);
y_real = (r_real*sin(theta)+y0_real);

% Plot results
figure; hold on;
plot(xcarthography, ycarthography, '.b');
plot(x,y,'-r');
plot(x_real,y_real,'-k');
daspect([1 1 1])
legend('LiDaR samples', 'Linear regression', 'Real obstacle')
xlabel('X [m]')
ylabel('Y [m]')
title('Linear regression of a circular obstacle using LiDaR measurments')
saveas(gcf,'Obstacle_linear_regression.png')

r_error = abs(r_real-r)
x0_error = abs(x0_real-x0)
y0_error = abs(y0_real-y0)

errors = [r_error x0_error y0_error];
max_error = max(errors)