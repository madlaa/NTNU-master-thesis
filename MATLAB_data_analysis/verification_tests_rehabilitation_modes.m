%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% README
%This .m file was used to produce the illustations for the different 
%force mode verification tests seen in the master thesis. 
%The code was reused for the different modes with some
%slight modifications between each analysis. All relevant logfiles are
%placed under the force_modes_tests folder. The logfiles used in the
%master theis is reperated under their respective names. Unused logfiles
%are filed in a separate folder.

%% Load data from file
%NB! - several extra parameters have been added at the end of each datasampling
%to better capture the safety- and force mode functionality testing.
%Doublecheck the length of each line in the logfiles to confirm relevant 'dim' length.
%Print MATLAB figure in PDF size: print('-fillpage', 'figure_title', '-dpdf')
fileID = fopen('datalogs/force_mode_tests/buoyancy_mode/drinking_1');
dim = 67; %time(1), q(6), s(6), etc..
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

%% Cartesian parameters
X = end_effector_coordiante_cartesian(:,1);
Y = end_effector_coordiante_cartesian(:,2);
Z = end_effector_coordiante_cartesian(:,3);

%% Rotate data from end-effector frame to cartesian frame
correlation_radius_vector_WF = zeros(size(correlation_radius_vector_TF));
correlation_height_vector_WF = zeros(size(correlation_height_vector_TF));
buoyancy_vector_WF = zeros(size(buoyancy_vector_TF));
random_disturbances_WF = zeros(size(random_disturbances_TF));
Forces_WF = zeros(size(Forces));
for n = 1:size(elapsTime)
    %Raduis correction vectors
    correlation_radius_vector_WF(n,1) = end_effector_orientation_TF(n,1:3)*correlation_radius_vector_TF(n,:)';
    correlation_radius_vector_WF(n,2) = end_effector_orientation_TF(n,4:6)*correlation_radius_vector_TF(n,:)';
    correlation_radius_vector_WF(n,3) = end_effector_orientation_TF(n,7:9)*correlation_radius_vector_TF(n,:)';
    %Height correction vectors
    correlation_height_vector_WF(n,1) = end_effector_orientation_TF(n,1:3)*correlation_height_vector_TF(n,:)';
    correlation_height_vector_WF(n,2) = end_effector_orientation_TF(n,4:6)*correlation_height_vector_TF(n,:)';
    correlation_height_vector_WF(n,3) = end_effector_orientation_TF(n,7:9)*correlation_height_vector_TF(n,:)';
    %Buoyancy vectors
    buoyancy_vector_WF(n,1) = end_effector_orientation_TF(n,1:3)*buoyancy_vector_TF(n,:)';
    buoyancy_vector_WF(n,2) = end_effector_orientation_TF(n,4:6)*buoyancy_vector_TF(n,:)';
    buoyancy_vector_WF(n,3) = end_effector_orientation_TF(n,7:9)*buoyancy_vector_TF(n,:)';
    %Random vectors
    random_disturbances_WF(n,1) = end_effector_orientation_TF(n,1:3)*random_disturbances_TF(n,:)';
    random_disturbances_WF(n,2) = end_effector_orientation_TF(n,4:6)*random_disturbances_TF(n,:)';
    random_disturbances_WF(n,3) = end_effector_orientation_TF(n,7:9)*random_disturbances_TF(n,:)';
    %Force vectors
    Forces_WF(n,1) = end_effector_orientation_TF(n,1:3)*Forces(n,:)';
    Forces_WF(n,2) = end_effector_orientation_TF(n,4:6)*Forces(n,:)';
    Forces_WF(n,3) = end_effector_orientation_TF(n,7:9)*Forces(n,:)';
end
%% Workspace parameters
min_r = 0.35;
max_r = 0.7;
min_height = -0.22;
max_height = 0.3;
cylinder_height = 0.52;

%% Plot perimeter testing 
figure('Name','Workspace perimeter testing');
% ================= Plot trajectory in three dimentional cartesian space =================
ax1 = subplot(2,1,1);
hold on;
plotting_scale = 20;
vector_scale = 1;
vector_of_interest_1 = correlation_height_vector_WF;
vector_of_interest_2 = correlation_radius_vector_WF;
vector_plotting_scale = 30;

%Trajectory of end-effector
trajectory = plot3(ax1, X(1:plotting_scale:end), Y(1:plotting_scale:end), Z(1:plotting_scale:end),'--m');
%Vectors of interest
height_vectors = quiver3(X(1:vector_plotting_scale:end), Y(1:vector_plotting_scale:end), Z(1:vector_plotting_scale:end), (vector_of_interest_1(1:vector_plotting_scale:end, 1))*vector_scale, (vector_of_interest_1(1:vector_plotting_scale:end, 2))*vector_scale, (vector_of_interest_1(1:vector_plotting_scale:end, 3)*vector_scale));
radius_vectors = quiver3(X(1:vector_plotting_scale:end), Y(1:vector_plotting_scale:end), Z(1:vector_plotting_scale:end), (vector_of_interest_2(1:vector_plotting_scale:end, 1))*vector_scale, (vector_of_interest_2(1:vector_plotting_scale:end, 2))*vector_scale, (vector_of_interest_2(1:vector_plotting_scale:end, 3)*vector_scale));
buoyancy_vectors = quiver3(X(1:plotting_scale:end), Y(1:plotting_scale:end), Z(1:plotting_scale:end), (buoyancy_vector_WF(1:plotting_scale:end, 1))*vector_scale, (buoyancy_vector_WF(1:plotting_scale:end, 2))*vector_scale, (buoyancy_vector_WF(1:plotting_scale:end, 3)*vector_scale));
random_vectors = quiver3(X(1:plotting_scale:end), Y(1:plotting_scale:end), Z(1:plotting_scale:end), (random_disturbances_WF(1:plotting_scale:end, 1))*vector_scale, (random_disturbances_WF(1:plotting_scale:end, 2))*vector_scale, (random_disturbances_WF(1:plotting_scale:end, 3)*vector_scale));
force_vectors = quiver3(X(1:plotting_scale:end), Y(1:plotting_scale:end), Z(1:plotting_scale:end), (Forces_WF(1:plotting_scale:end, 1))*vector_scale, (Forces_WF(1:plotting_scale:end, 2))*vector_scale, (Forces_WF(1:plotting_scale:end, 3)*vector_scale));

%Starting position
start = scatter3(X(1), Y(1), Z(1), 'filled', 'g');

%End position
stop = scatter3(X(end), Y(end), Z(end), 'filled', 'r');

%Virtual workspace perimeter
[A,B,C] = cylinder(min_r, 250);
[D,E,F] = cylinder(max_r, 250);
min_cylinder = surf(A,B,C*cylinder_height+min_height, 'EdgeColor', 'b', 'FaceAlpha', .1);
max_cylinder = surf(D,E,F*cylinder_height+min_height, 'FaceAlpha',.1);
shading interp
max_circle = fill3(D(:),E(:), ones(size(F(:)))*max_height, 'y', 'FaceAlpha', .15, 'EdgeColor', 'non');
min_circle = fill3(D(:),E(:), ones(size(F(:)))*min_height, 'b', 'FaceAlpha', .1, 'EdgeColor', 'non');


%Actual workspace perimeter (r_sphere = 0.85[m])
%[x,y,z] = sphere;
%surf(x*0.85,y*0.85,z*0.85, 'FaceAlpha',.05)

view(45, 15) %45 135 225 315
xlabel('\fontsize{15} X-axis')
ylabel('\fontsize{15} Y-axis')
zlabel('\fontsize{15} Z-axis')
xlim([min(X) max(X)])
ylim([min(Y) max(Y)])
zlim([min(Z) max(Z)+0.05])
%title('\fontsize{15} Trajectory in cartesian space');
ax1 = legend([trajectory, force_vectors, start, stop, min_cylinder], {'\fontsize{15} Trajectory', '\fontsize{15} Force vectors', '\fontsize{15} Start','\fontsize{15} End', '\fontsize{15} Boundaries'}, 'Location','northwest');
%title(cartesian_space, '\fontsize{15} Legend')
grid on;
hold off;

% ================= Plot force readings =================
ax2 = subplot(2,1,2);
%Detect and highlight when outside soft-boundry area
m = 1;
color = 'none';
for n = 1:size(elapsTime)
    radius_current = sqrt(power(X(n), 2) + power(Y(n), 2));
    if (Z(n) >= max_height) || (Z(n) <= min_height)
        if(strcmp(color, 'none'))
            color = 'r';
        end
        
        if (~strcmp(color, 'r'))
            plot(elapsTime(m:n,1), Forces_WF(m:n,1), 'g')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,2), 'b')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,3), 'm')
            hold on;
            m = n;
            color = 'r';
        end
    elseif (radius_current <= min_r) || (radius_current >= max_r)
        if(strcmp(color, 'none'))
            color = 'r';
        end
        
        if (~strcmp(color, 'r'))
            plot(elapsTime(m:n,1), Forces_WF(m:n,1), 'g')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,2), 'b')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,3), 'm')
            hold on;
            m = n;
            color = 'r';
        end
    else
        if(strcmp(color, 'none'))
            color = 'b';
        end
            
        if (~strcmp(color, 'b'))
            plot(elapsTime(m:n,1), Forces_WF(m:n,1), 'r')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,2), 'r')
            hold on;
            plot(elapsTime(m:n,1), Forces_WF(m:n,3), 'r')
            hold on;
            m = n;
            color = 'b';
        end
    end
end
if (m<n && strcmp(color, 'r'))
    plot(elapsTime(m:n,1), Forces_WF(m:n,1), 'r')
    hold on;
    plot(elapsTime(m:n,1), Forces_WF(m:n,2), 'r')
    hold on;
    plot(elapsTime(m:n,1), Forces_WF(m:n,3), 'r')
    hold on;
    m = size(elapsTime(:,1));
elseif (m<n && strcmp(color, 'b'))
    plot(elapsTime(m:n,1), Forces_WF(m:n,1), 'g')
    hold on;
    plot(elapsTime(m:n,1), Forces_WF(m:n,2), 'b')
    hold on;
    plot(elapsTime(m:n,1), Forces_WF(m:n,3), 'm')
    hold on;
    m = size(elapsTime(:,1));
end

%q6 = plot(elapsTime, -q(:,6));
%title('\fontsize{15} Force data');
ax2 = legend('\fontsize{15} Fx', '\fontsize{15} Fy', '\fontsize{15} Fz', 'Location','east');%,'\fontsize{15} Fy', '\fontsize{15} Fz', 'Location','east');
%title(ax2,'\fontsize{15} Force inputs')
xlabel('\fontsize{15} Time [s]')
ylabel('\fontsize{15} Force [N]')
grid on;
hold off;
