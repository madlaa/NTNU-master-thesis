clc;
close all;
clear all;

%Angle theta in [rad] for i={1,...,6}
q1 = sym('q1');
q2 = sym('q2');
q3 = sym('q3');
q4 = sym('q4');
q5 = sym('q5');
q6 = sym('q6');
%Distance d in [mm] for joints i={1,...,6}
d1 = 89.459;
d2 = 0;
d3 = 0;
d4 = 109.15;
d5 = 94.65;
d6 = 82.3;

%Distance a in [mm] for i={1,...,6}
a1 = 0;
a2 = -425;
a3 = -392.25;
a4 = 0;
a5 = 0;
a6 = 0;
%Angle alpha for i={1,...,6}
alpha1 = pi/2;
alpha2 = 0;
alpha3 = 0;
alpha4 = pi/2;
alpha5 = -pi/2;
alpha6 = 0;

A1 = [cos(q1) 0 sin(q1) 0;
       sin(q1) 0 -cos(q1) 0;
       0 1 0 d1;
       0 0 0 1];

A2 = [cos(q2) -sin(q2) 0 a2;
       sin(q1) cos(q2) 0 0;
       0 0 1 0;
       0 0 0 1];
   
A3 = [cos(q3) -sin(q3) 0 a3;
       sin(q3) cos(q3) 0 0;
       0 0 1 0;
       0 0 0 1];

A4 = [cos(q4) 0 sin(q4) 0;
       sin(q4) 0 -cos(q4) 0;
       0 1 0 d4;
       0 0 0 1];
   
A5 = [cos(q5) 0 -sin(q5) 0;
       sin(q5) 0 cos(q5) 0;
       0 1 0 d5;
       0 0 0 1];
   
A6 = [cos(q6) -sin(q6) 0 0;
       sin(q6) cos(q6) 0 0;
       0 0 1 d6;
       0 0 0 1];
   
T1 = A1;
T2 = A1*A2;
T3 = A1*A2*A3;
T4 = A1*A2*A3*A4;
T5 = A1*A2*A3*A4*A5;
T6 = A1*A2*A3*A4*A5*A6;