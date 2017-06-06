%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Load data from files
fileID = fopen('datalogs/safety_testing/max_force_Fx');
dim = 70; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

fileID1 = fopen('datalogs/safety_testing/max_force_Fy');
raw_data1 = textscan(fileID1, data_format); %Remember to delete any incomplete log entries in the final row.
data1 = cell2mat(raw_data1); %Convert cell array
fclose(fileID1);

fileID2 = fopen('datalogs/safety_testing/max_force_Fz');
raw_data2 = textscan(fileID2, data_format); %Remember to delete any incomplete log entries in the final row.
data2 = cell2mat(raw_data2); %Convert cell array
fclose(fileID2);

fileID3 = fopen('datalogs/safety_testing/max_torque_8Nm_Rx');
raw_data3 = textscan(fileID3, data_format); %Remember to delete any incomplete log entries in the final row.
data3 = cell2mat(raw_data3); %Convert cell array
fclose(fileID3);

fileID4 = fopen('datalogs/safety_testing/max_torque_8Nm_Ry');
raw_data4 = textscan(fileID4, data_format); %Remember to delete any incomplete log entries in the final row.
data4 = cell2mat(raw_data4); %Convert cell array
fclose(fileID4);

fileID5 = fopen('datalogs/safety_testing/max_torque_8Nm_Rz');
raw_data5 = textscan(fileID5, data_format); %Remember to delete any incomplete log entries in the final row.
data5 = cell2mat(raw_data5); %Convert cell array
fclose(fileID5);

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

% Distribute data1 into usefull matricies
elapsTime1 = data1(:,1);
Forces1 = data1(:, 20:22);
Torques1 = data1(:, 23:25);

% Distribute data2 into usefull matricies
elapsTime2 = data2(:,1);
Forces2 = data2(:, 20:22);
Torques2 = data2(:, 23:25);

% Distribute data3 into usefull matricies
elapsTime3 = data3(:,1);
Forces3 = data3(:, 20:22);
Torques3 = data3(:, 23:25);

% Distribute data4 into usefull matricies
elapsTime4 = data4(:,1);
Forces4 = data4(:, 20:22);
Torques4 = data4(:, 23:25);

% Distribute data5 into usefull matricies
elapsTime5 = data5(:,1);
Forces5 = data5(:, 20:22);
Torques5 = data5(:, 23:25);

%% Plot force sensor input
figure('Name','Emergency shutdown testing');
ax1 = subplot(2,1,1);
hold on;
%Forces
F_x = plot(ax1, elapsTime, -Forces(:,1), 'g');
F_y = plot(ax1, elapsTime1, -Forces1(:,2), 'b');
F_z = plot(ax1, elapsTime2, -Forces2(:,3), 'm');
F_limit = plot(ax1, elapsTime1, ones(size(Forces1(:,1)))*50, '--r');
%End line
Fx_endline = min(Forces(end,:)):50;
Fy_endline = min(Forces1(end,:)):50;
Fz_endline = min(Forces2(end,:)):50;
Fx_stop = plot(ones(size(Fx_endline))*elapsTime(end,1), Fx_endline, '--r');
Fy_stop = plot(ones(size(Fy_endline))*elapsTime1(end,1), Fy_endline, '--r');
Fz_stop = plot(ones(size(Fz_endline))*elapsTime2(end,1), Fz_endline, '--r');
ax1 = legend([F_x, F_y, F_z, F_limit], {'\fontsize{15} F_x', '\fontsize{15} F_y','\fontsize{15} F_z', '\fontsize{15} Force limit'}, 'Location','east');
xlabel('Time [s]')
ylabel('Force [N]')
xlim([0, 4.5])
ylim([-20, 60])

grid on;
hold off;

%% Plot torque sensor input
ax2 = subplot(2,1,2);
hold on;
%Torques
T_x = plot(ax2, elapsTime3,Torques3(:,1), 'g');
T_y = plot(ax2, elapsTime4,-Torques4(:,2), 'b');
T_z = plot(ax2, elapsTime5,Torques5(:,3), 'm');
T_limit = plot(ax2, elapsTime4, ones(size(Torques4(:,1)))*8, '--r');
%End line
Tx_endline = min(Forces(end,:)):9;
Ty_endline = min(Forces1(end,:)):9;
Tz_endline = min(Forces2(end,:)):9;
Fx_stop = plot(ones(size(Tx_endline))*elapsTime3(end,1), Tx_endline, '--r');
Fy_stop = plot(ones(size(Ty_endline))*elapsTime4(end,1), Ty_endline, '--r');
Fz_stop = plot(ones(size(Tz_endline))*elapsTime5(end,1), Tz_endline, '--r');
ax2 = legend([T_x, T_y, T_z, T_limit], {'\fontsize{15} T_x', '\fontsize{15} T_y','\fontsize{15} T_z', '\fontsize{15} Torque limit'}, 'Location','east');
xlabel('Time [s]')
ylabel('Torque [Nm]')
xlim([0, 7.5])
ylim([-5, 12])
grid on;
hold off;
