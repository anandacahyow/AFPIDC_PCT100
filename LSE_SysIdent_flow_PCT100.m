% ALGORITMA ESTIMASI PARAMETER PLANT FLOW CONTROL PCT - 100
% MENGGUNAKAN ARX MODEL LEAST SQUARE ESTIMATION

clear;clc;close all;
% Experimental Data Initialisation
PCT = importdata('openloop_PCT100data.csv');
t = PCT.data(:,1);
u = PCT.data(:,2);
y = PCT.data(:,3);
%Parameter Initialisation
na = 2;
nb = 1;
nd = 3;
%% System Identification

sys = arx([y,u],[na nb nd],'Ts',0.1)

%% LSE

y1 = [zeros(1,na) y(1:length(u)-na)']';
y2 = [zeros(1,nb) y(1:length(u)-nb)']';
u3 = [zeros(1,nd) u(1:length(u)-nd)']';;

phi = [y1 y2 u3];

theta = inv(phi'*phi)*phi'*y
a1 = theta(1);
a2 = theta(2);
b1 = theta(3);

%% ARX model to Transfer Function

B = [b1];
A = [1 -a2 -a1];
G = tf(B,A,0.1,'InputDelay',3)

[y_est,tt] = step(100*G,t(end));

%% Tes PID

C = pid(0.72,1.44,0.09,'Ts',0.1)
H = feedback(80*C*G,1);

figure(1)
rlocus(H)
grid on

K = 20;
TF = feedback(K*C*G,1);
[y_pid,tt] = step(TF,t(end));

%% Plot ARX model vs Experimental Data

% ARX model vs real data
figure(2)
plot(tt,y)
hold on
plot(tt,y_est)
plot(tt,y_pid)
legend('PCT - 100 flow','Estimated ARX','PID PCT100')
grid on
title('Flow Measured Value of Experimantal Data vs ARX Model')
xlabel('time(s)')
ylabel('flow(L/min)')

disp('==== Time Domain Spec. Experimental Data ====')
stepinfo(y)
disp('==== Time Domain Spec. Estimated ARX Model ====')
stepinfo(y_est)
disp('==== Time Domain Spec. with PID Controller ====')
stepinfo(y_pid)
%% Tes Combined Response

PCT_PID = importdata('ZN2_PID_sp_tes.csv');