%% TTK4900 Teknisk kybernetikk - Master thesis
%Mads Johan Laastad
%Spring 2017
clc;
close all;
clear all;
%% Workspace parameters
min_r = 0.35;
max_r = 0.7;
min_height = -0.22;
max_height = 0.3;
cylinder_height = 0.52;

%% Plot perimeter testing 
figure('Name','Virtual workspace');
% ================= Plot front view of virtual workspace =================
ax1 = subplot(1,2,1);
hold on;

%Virtual workspace perimeter
[A,B,C] = cylinder(min_r, 250);
[D,E,F] = cylinder(max_r, 250);
shading interp
min_cylinder = surf(A,B,C*cylinder_height+min_height, 'EdgeColor', 'b', 'FaceAlpha', .2);
max_cylinder = surf(D,E,F*cylinder_height+min_height, 'FaceAlpha',.2);
shading flat
max_circle = fill3(D(:),E(:), ones(size(F(:)))*max_height, 'y', 'FaceAlpha', .2, 'EdgeColor', 'non');
min_circle = fill3(D(:),E(:), ones(size(F(:)))*min_height, 'b', 'FaceAlpha', .2, 'EdgeColor', 'non');

% Actual workspace perimeter (r_sphere = 0.85[m])
[x,y,z] = sphere;
shading interp
surf(x*0.85,y*0.85,z*0.85, 'EdgeColor', 'none', 'FaceAlpha',.1)

view(45, 10) %45 135 225 315
xlabel('\fontsize{15} X-axis')
ylabel('\fontsize{15} Y-axis')
zlabel('\fontsize{15} Z-axis')
%title('\fontsize{15} Trajectory in cartesian space');
%ax1 = legend([ min_cylinder, max_cylinder], {'\fontsize{15} Trajectory', '\fontsize{15} Start','\fontsize{15} End', '\fontsize{15} Workspace boundary'}, 'Location','west');
%title(cartesian_space, '\fontsize{15} Legend')
grid on;
hold off;

% ================= Plot tilted view of virtual workspace =================
ax2 = subplot(1,2,2);
hold on;

%Virtual workspace perimeter
[A2,B2,C2] = cylinder(min_r, 250);
[D2,E2,F2] = cylinder(max_r, 250);
shading flat
min_cylinder2 = surf(A2,B2,C2*cylinder_height+min_height, 'EdgeColor', 'b', 'FaceAlpha', .2);
max_cylinder2 = surf(D2,E2,F2*cylinder_height+min_height, 'FaceAlpha',.2);
shading interp
max_circle2 = fill3(D2(:),E2(:), ones(size(F2(:)))*max_height, 'y', 'FaceAlpha', .2, 'EdgeColor', 'non');
min_circle2 = fill3(D2(:),E2(:), ones(size(F2(:)))*min_height, 'b', 'FaceAlpha', .2, 'EdgeColor', 'non');
% Actual workspace perimeter (r_sphere = 0.85[m])
[x2,y2,z2] = sphere;
shading interp
surf(x2*0.85,y2*0.85,z2*0.85, 'EdgeColor', 'none', 'FaceAlpha',.1)

view(45, 30) %45 135 225 315
xlabel('\fontsize{15} X-axis')
ylabel('\fontsize{15} Y-axis')
zlabel('\fontsize{15} Z-axis')
%title('\fontsize{15} Trajectory in cartesian space');
%ax2 = legend([trajectory, start, stop, max_cylinder2], {'\fontsize{15} Trajectory', '\fontsize{15} Start','\fontsize{15} End', '\fontsize{15} Workspace boundary'}, 'Location','west');
%title(cartesian_space, '\fontsize{15} Legend')
grid on;
hold off;