One P-controller for forces and one P-controller for torques. Kontroller gain parameters:
Kp_F = 0.005;
Kp_T = 0.4;

All four tests last for 30s. The 2-plane mode is initiated slightly offset from the horizontal plane due to human error.

The 2-plane mode is tested for a functional reach and a circle movement.

The random mode is tested with all three random axis vectors at 100% (input: 100 100 100). Random mode is tested for a functional reach and a circle movement.

The resistance mode is tested with all six degrees of freedom at 50% resistance (input: 50 50 50 50 50 50). Resistance mode is tested for a functional reach movement and a drinking task.

The buoyancy mode is tested with a buoyancy vector set to 15N (input: 15). The buoyancy mode is tested for a functional reach movement and a drinking task.

============================ MATLAB EXAMPLE SCRIPT ================================
fileID = fopen('datalogs/force_mode_tests/buoyancy_mode/drinking_2');
dim = 67; %time(1), q(6), s(6), etc..
data_format = repmat('%f ', 1, dim);
raw_data = textscan(fileID, data_format); %Remember to delete any incomplete log entries in the final row.
[N, M] = size(raw_data{1,1});
data = cell2mat(raw_data); %Convert cell array
fclose(fileID);

%All four tests have 67 data samling points:
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
%OBS! biasFT = REMOVED
biasForce = data(:, 38:40);
bias_tool_TF = data(:, 41:43);
end_effector_coordiante_cartesian = data(:, 44:46);
%Extra data sampling points
random_disturbances_TF = data(:, 47:49);
buoyancy_vector_TF = data(:, 50:52);
correlation_radius_vector_TF = data(:, 53:55);
correlation_height_vector_TF = data(:, 56:58);
end_effector_orientation_TF = data(:, 59:67);

