%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from file
fileID = fopen('data/logs/forcelog');
dim = 31; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);
%[N,M] = size(data); %N number of discreate datasamples generated

% Distribute data into usefull matricies
elapsTime = data(:,1);
speed = data(:, 2:7);
q = data(:, 8:13);
u = data(:, 14:16);
errors = data(:, 17:19);
torques = data(:, 20:22);
forces = data(:, 23:25);
biasFT = data(:, 26:28);
biasTF = data(:, 29:31);
%% Plot joint angles
figure('Name','Joint angles');
plot(elapsTime,q(:,1),elapsTime,q(:,2),elapsTime,q(:,3),elapsTime,q(:,4), elapsTime,q(:,5), elapsTime,q(:,6))
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')
title('Joint angles');
xlabel('Elapsed time [s]')
ylabel('Joint angles [rad]')
grid on;

%% Plot control inputs
figure('Name','Control inputs');
plot(elapsTime,u(:,1),elapsTime,u(:,2),elapsTime,u(:,3))
legend('u_{Fx}','u_{Fy}','u_{Fz}')
title('Control inputs');
xlabel('Elapsed time [s]')
ylabel('Control input')
grid on;

%% Plot torque sensor input
figure('Name','Torque sensor input');
plot(elapsTime,torques(:,1),elapsTime,torques(:,2),elapsTime,torques(:,3))
legend('T_x','T_y','T_z')
title('Torque sensor input');
xlabel('Elapsed time [s]')
ylabel('Torque sensor input [Nm]')
grid on;

%% Plot force sensor input
figure('Name','Force sensor input');
plot(elapsTime,forces(:,1),elapsTime,forces(:,2),elapsTime,forces(:,3))
legend('F_x','F_y','F_z')
title('Force sensor input');
xlabel('Elapsed time [s]')
ylabel('Torque sensor input [N]')
grid on;

%% Plot force sensor input
figure('Name','Errors');
plot(elapsTime,errors(:,1),elapsTime,errors(:,2),elapsTime,errors(:,3))
legend('Error_{Fx}','Error_{Fy}','Error_{Fz}')
title('Errors');
xlabel('Elapsed time [s]')
ylabel('Torque sensor input [N]')
grid on;

%% Plot Tool frame input
figure('Name','Tool frame bias (biasTF)');
plot(elapsTime,biasTF(:,3),elapsTime,biasTF(:,2),elapsTime,biasTF(:,1)) 
hold on;
plot(elapsTime,biasFT(:,1),elapsTime,biasFT(:,2),elapsTime,biasFT(:,3)) 
hold off;
legend('biasTF_{Fx}','biasTF_{Fy}','biasTF_{Fz}', 'biasFT_{Fx}','biasFT_{Fy}','biasFT_{Fz}')
title('Tool frame bias (biasTF)');
xlabel('Elapsed time [s]')
ylabel('Bias tool frame force input [N]')
grid on;

% %% Plot force frame constant
% figure('Name','Force frame bias (biasFT)');
% plot(elapsTime,biasFT(:,3),elapsTime,biasFT(:,2),elapsTime,biasFT(:,1))
% legend('biasFT_{Fx}','biasFT_{Fy}','biasFT_{Fz}')
% title('Force frame bias (biasFT)');
% xlabel('Elapsed time [s]')
% ylabel('Bias tool frame force input [N]')
% grid on;
