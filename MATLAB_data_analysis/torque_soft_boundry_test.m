%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% README
%Time did not allow to properly test possible joint soft- and hard boundies
%in the external controller. These results are threfore not presented in 
%the master thesis.
%% Load data from files
fileID = fopen('datalogs/safety_testing/soft_torque_Rz_1');
dim = 70; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

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
end_effector_coordinates = data(:, 47:49);
%Extra data samplings
random_disturbances = data(:, 50:52);
buoyancy_vector = data(:, 53:55);
correlation_radius_vector = data(:, 56:58);
correlation_height_vector = data(:, 59:61);
end_effector_orientation_TF = data(:, 62:70);

%% Plot joint angles
figure('Name','Emergency shutdown testing');
ax1 = subplot(2,1,1);
hold on;
%Joint angles
% q1 = plot(ax1, elapsTime, q(:,1));
% q2 = plot(ax1, elapsTime, q(:,2));
% q3 = plot(ax1, elapsTime, q(:,3));
% q4 = plot(ax1, elapsTime, q(:,4));
% q5 = plot(ax1, elapsTime, q(:,5));
q6 = plot(ax1, elapsTime, -q(:,6));
q_limit = plot(ax1, elapsTime, ones(size(q(:,1)))*(pi+q(1,6)), '--r');
q_soft = plot(ax1, elapsTime, ones(size(q(:,1)))*(pi/2+q(1,6)), '--b');
%End line
q6_endline = min(q(end,:)):-q(end,6)+0.7;%(pi+q(1,6)+0.2);
% Fx_endline = min(Forces(end,:)):50;
% Fy_endline = min(Forces1(end,:)):50;
% Fz_endline = min(Forces2(end,:)):50;
q6_stop = plot(ones(size(q6_endline))*elapsTime(end,1), q6_endline, '--r');
% Fx_stop = plot(ones(size(Fx_endline))*elapsTime(end,1), Fx_endline, '--r');
% Fy_stop = plot(ones(size(Fy_endline))*elapsTime1(end,1), Fy_endline, '--r');
% Fz_stop = plot(ones(size(Fz_endline))*elapsTime2(end,1), Fz_endline, '--r');
%[q1, q2, q3, q4, q5, q6, q_limit]
ax1 = legend([q6, q_limit, q_soft], {'\fontsize{15} q6', '\fontsize{15} Hard boundary', '\fontsize{15} Soft boundary'}, 'Location','east');
xlabel('\fontsize{15} Time [s]')
ylabel('\fontsize{15} Joint angle [rad]')
xlim([0, 13.5])
ylim([-2, 5])

grid on;
hold off;

%% Plot torque sensor input
ax2 = subplot(2,1,2);
hold on;
%Torques
% T_x = plot(ax2, elapsTime,Torques(:,1), 'g');
% T_y = plot(ax2, elapsTime,Torques(:,2), 'b');
T_z = plot(ax2, elapsTime,-Torques(:,3), 'm');
T_limit = plot(ax2, elapsTime, ones(size(Torques(:,1)))*8, '--r');
%End line
% Tx_endline = min(Forces(end,:)):9;
% Ty_endline = min(Forces1(end,:)):9;
% Tz_endline = min(Forces2(end,:)):9;
% Fx_stop = plot(ones(size(Tx_endline))*elapsTime3(end,1), Tx_endline, '--r');
% Fy_stop = plot(ones(size(Ty_endline))*elapsTime4(end,1), Ty_endline, '--r');
% Fz_stop = plot(ones(size(Tz_endline))*elapsTime5(end,1), Tz_endline, '--r');
ax2 = legend([T_z, T_limit], {'\fontsize{15} T_z', '\fontsize{15} Torque limit'}, 'Location','east');
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim([0, 13.5])
ylim([-1, 9])
grid on;
hold off;
