%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from file
fileID = fopen('tool_bias_Rx');
dim = 46; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

fileID1 = fopen('tool_bias_Ry');
%dim = 46; %time(1), q(6), s(6), etc..
%data_format = repmat('%f ', 1, dim);
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
[N1, M1] = size(raw_data1{1,1});
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

fileID2 = fopen('tool_bias_Rz');
%dim = 46; %time(1), q(6), s(6), etc..
%data_format = repmat('%f ', 1, dim);
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
[N2, M2] = size(raw_data2{1,1});
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

% Distribute data into usefull matricies
elapsTime = data(:,1);
speed = data(:, 2:7);
q = data(:, 8:13);
rawFTdata = data(:, 14:19);
Forces = data(:, 20:22);
Torques = data(:, 23:25);
errors_F = data(:, 26:28);
errors_T = data(:, 29:31);
u_F = data(:, 32:34);
u_T = data(:, 35:37);
biasFT = data(:, 38:40);
biasForce = data(:, 41:43);
bias_tool_TF = data(:, 44:46);

elapsTime1 = data1(:,1);
Forces1 = data1(:, 20:22);
elapsTime2 = data2(:,1);
Forces2 = data2(:, 20:22);

%% Plot force sensor input
figure('Name','Tool bias test force sensor data');
subplot(3,1,1)
plot(elapsTime,Forces(:,1),elapsTime,Forces(:,2),elapsTime,Forces(:,3))
legend('F_x','F_y','F_z')
title('Tool bias test - Rotation in X, Y, Z-axis');
ylabel('Rx - Force sensor input [N]')
grid on;

subplot(3,1,2)
plot(elapsTime1,Forces1(:,1),elapsTime1,Forces1(:,2),elapsTime1,Forces1(:,3))
legend('F_x','F_y','F_z')
ylabel('Ry - Force sensor input [N]')
grid on;

subplot(3,1,3)
plot(elapsTime2,Forces2(:,1),elapsTime2,Forces2(:,2),elapsTime2,Forces2(:,3))
legend('F_x','F_y','F_z')
xlabel('Elapsed time [s]')
ylabel('Rz - Force sensor input [N]')
grid on;

% %% Plot joint speeds
% figure('Name','Joint speeds');
% plot(elapsTime,speed(:,1),elapsTime,speed(:,2),elapsTime,speed(:,3),elapsTime,speed(:,4), elapsTime,speed(:,5), elapsTime,speed(:,6))
% legend('qd_1', 'qd_2', 'qd_3', 'qd_4', 'qd_5', 'qd_6')
% title('Joint speeds');
% xlabel('Elapsed time [s]')
% ylabel('Joint speeds [rad/s]')
% grid on;

% %% Plot joint angles
% figure('Name','Joint angles');
% plot(elapsTime,q(:,1),elapsTime,q(:,2),elapsTime,q(:,3),elapsTime,q(:,4), elapsTime,q(:,5), elapsTime,q(:,6))
% legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')
% title('Joint angles');
% xlabel('Elapsed time [s]')
% ylabel('Joint angles [rad]')
% grid on;

% %% Plot raw sensor inputs
% figure('Name','Raw sensor inputs');
% plot(elapsTime,rawFTdata(:,1),elapsTime,rawFTdata(:,2),elapsTime,rawFTdata(:,3),elapsTime,rawFTdata(:,4), elapsTime,rawFTdata(:,5), elapsTime,rawFTdata(:,6))
% legend('raw_{Fx}', 'raw_{Fy}', 'raw_{Fz}', 'raw_{Tx}', 'raw_{Ty}', 'raw_{Tz}')
% title('Raw sensor inputs');
% xlabel('Elapsed time [s]')
% ylabel('Raw sensor inputs [N]')
% grid on;

% %% Plot torque sensor input
% figure('Name','Torque sensor input');
% plot(elapsTime,Torques(:,1),elapsTime,Torques(:,2),elapsTime,Torques(:,3))
% legend('T_x','T_y','T_z')
% title('Torque sensor input');
% xlabel('Elapsed time [s]')
% ylabel('Torque sensor input [Nm]')
% grid on;

% %% Plot force control inputs
% figure('Name','Force control inputs');
% plot(elapsTime,u_F(:,1),elapsTime,u_F(:,2),elapsTime,u_F(:,3))
% legend('u_{Fx}','u_{Fy}','u_{Fz}')
% title('Control inputs');
% xlabel('Elapsed time [s]')
% ylabel('Control input')
% grid on;

% %% Plot torque control inputs
% figure('Name','Torque control inputs');
% plot(elapsTime,u_T(:,1),elapsTime,u_T(:,2),elapsTime,u_T(:,3))
% legend('u_{Fx}','u_{Fy}','u_{Fz}')
% title('Control inputs');
% xlabel('Elapsed time [s]')
% ylabel('Control input')
% grid on;

% %% Plot force controller errors
% figure('Name','Force controller errors');
% plot(elapsTime,errors_F(:,1),elapsTime,errors_F(:,2),elapsTime,errors_F(:,3))
% legend('Error_{Fx}','Error_{Fy}','Error_{Fz}')
% title('Force controller errors');
% xlabel('Elapsed time [s]')
% ylabel('Errors [N]')
% grid on;

% %% Plot torque controller errors
% figure('Name','Torque controller errors');
% plot(elapsTime,errors_T(:,1),elapsTime,errors_T(:,2),elapsTime,errors_T(:,3))
% legend('Error_{Tx}','Error_{Ty}','Error_{Tz}')
% title('Torque controller errors');
% xlabel('Elapsed time [s]')
% ylabel('Errors [N]')
% grid on;

% %% Plot tool frame bias vs. force frame bias
% figure('Name','tool frame bias vs. force frame bias');
% plot(elapsTime,biasForce(:,1), '--o', elapsTime,biasForce(:,2), '--', elapsTime,biasForce(:,3)) 
% hold on;
% plot(elapsTime,biasFT(:,1), '--o', elapsTime,biasFT(:,2), '--', elapsTime,biasFT(:,3)) 
% hold off;
% legend('biasForce_{Fx}','biasForce_{Fy}','biasForce_{Fz}', 'biasFT_{Fx}','biasFT_{Fy}','biasFT_{Fz}')
% title('tool frame bias vs. force frame bias');
% xlabel('Elapsed time [s]')
% ylabel('Bias [N]')
% grid on;

% %% Plot bias_tool_TF vs. forces
% figure('Name','tool frame bias vs. force frame bias');
% plot(elapsTime,biasForce(:,1), '--o', elapsTime,biasForce(:,2), '--', elapsTime,biasForce(:,3)) 
% hold on;
% plot(elapsTime,rawFTdata(:,1), '--o',elapsTime,rawFTdata(:,2), '--',elapsTime,rawFTdata(:,3)) 
% hold off;
% legend('biasForce_{Fx}','biasForce_{Fy}','biasForce_{Fz}', 'raw_{Fx}','raw_{Fy}','raw_{Fz}')
% title('tool frame bias vs. force frame bias');
% xlabel('Elapsed time [s]')
% ylabel('Bias [N]')
% grid on;


% %% Plot force frame bias
% figure('Name','Force frame bias (biasFT)');
% plot(elapsTime,biasFT(:,3),elapsTime,biasFT(:,2),elapsTime,biasFT(:,1))
% legend('biasFT_{Fx}','biasFT_{Fy}','biasFT_{Fz}')
% title('Force frame bias (biasFT)');
% xlabel('Elapsed time [s]')
% ylabel('Bias tool frame force input [N]')
% grid on;

% %% Plot bias_tool_TF
% figure('Name','bias_tool_TF');
% plot(elapsTime,bias_tool_TF(:,1),elapsTime,bias_tool_TF(:,2),elapsTime,bias_tool_TF(:,3))
% legend('bias_{}tool{}_TF_{Fx}','bias_{}tool_{}TF_{Fy}','bias_{}tool_{}TF_{Fz}')
% title('bias_tool_TF');
% xlabel('Elapsed time [s]')
% ylabel('Bias tool frame force input [N]')
% %xlim([0 5])
% %ylim([0 5])
% grid on;
