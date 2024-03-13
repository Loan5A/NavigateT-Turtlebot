clear all; close all; clc;

% Initialize the time stamp, the state and input reference :
t = [];
xi_ref = [];
u_ref = [];

% define the initial and final conditions :
xi = [0;0;0];
ui = [0;0];
xf = [1.2;-0.9;pi/2];
uf = [0;0];
T = 60;
h = 0.1;

% Contruct the matrix M & b
M=[1 0 0   0     0      0      0 0 0   0     0      0;
   0 1 0   0     0      0      0 0 0   0     0      0;
   0 0 2   0     0      0      0 0 0   0     0      0;
   0 0 0   0     0      0      1 0 0   0     0      0;
   0 0 0   0     0      0      0 1 0   0     0      0;
   0 0 0   0     0      0      0 0 2   0     0      0;
   1 T T^2 T^3   T^4    T^5    0 0 0   0     0      0;
   0 1 2*T 3*T^2 4*T^3  5*T^4  0 0 0   0     0      0;
   0 0 2   6*T   12*T^2 20*T^3 0 0 0   0     0      0;
   0 0 0   0     0      0      1 T T^2 T^3   T^4    T^5;
   0 0 0   0     0      0      0 1 2*T 3*T^2 4*T^3  5*T^4;
   0 0 0   0     0      0      0 0 2   6*T   12*T^2 20*T^3];


b=[xi(1) ui(1)*cos(xi(3)) -ui(1)*sin(xi(3))*ui(2)...
   xi(2) ui(1)*sin(xi(3)) ui(1)*cos(xi(3))*ui(2)...
   xf(1) uf(1)*cos(xf(3)) -uf(1)*sin(xf(3))*uf(2)...
   xf(2) uf(1)*sin(xf(3)) uf(1)*cos(xf(3))*uf(2)]';

a=inv(M)*b

z1 = @(t) (a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3+a(5)*t.^4+a(6)*t.^5);
z2 = @(t) (a(7)+a(8)*t+a(9)*t.^2+a(10)*t.^3+a(11)*t.^4+a(12)*t.^5);

zd1 = @(t) (a(2)+2*a(3)*t+3*a(4)*t.^2+4*a(5)*t.^3+5*a(6)*t.^4);
zd2 = @(t) (a(8)+2*a(9)*t+3*a(10)*t.^2+4*a(11)*t.^3+5*a(12)*t.^4);

zdd1 = @(t) (2*a(3)+6*a(4)*t+12*a(5)*t.^2+20*a(6)*t.^3);
zdd2 = @(t) (2*a(9)+6*a(10)*t+12*a(11)*t.^2+20*a(12)*t.^3);

X = @(t) ([feval(z1, t); feval(z2, t); atan2(feval(zd2, t), feval(zd1, t))]);
U = @(t) ([sqrt(feval(zd1, t).^2+feval(zd2, t).^2); atan2(feval(zdd2, t), feval(zdd1, t))]);

% Compute the state and input reference
tt = h:h:T;
xi_ref=[xi_ref feval(X, tt)];
xi_ref(3,:) = mod(xi_ref(3,:),2*pi)

u_ref=[u_ref feval(U, tt)];

figure;
plot(xi_ref(1,:), xi_ref(2,:));

figure;hold on;
plot(u_ref(1,:));
plot(u_ref(2,:));
legend('Linear velocity','Angular velocity')

figure;hold on;
plot(tt, xi_ref(1,:));
plot(tt, xi_ref(2,:));
plot(tt, xi_ref(3,:));
legend('X','Y','\theta')
file = 'trajectory.csv'

trajectory = xi_ref'
writematrix(round(trajectory, 6), file, "Delimiter",";")
%dlmwrite(file, trajectory, ',' 'precision', 4);


