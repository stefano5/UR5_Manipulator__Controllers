clear all
close all

addpath matrix_simulink
addpath regressore


%% load robot
mdl_ur5
% ur5

%% get masses
m1 = ur5.links(1,1).m;
m2 = ur5.links(1,2).m;
m3 = ur5.links(1,3).m;
m4 = ur5.links(1,4).m;
m5 = ur5.links(1,5).m;
m6 = ur5.links(1,6).m;

%% set intertia 
% https://webthesis.biblio.polito.it/15519/1/tesi.pdf, pag 19

ur5.links(1,1).I = diag([0.0067,0.0064,0.0067]);
ur5.links(1,2).I = diag([0.0149,0.3564,0.3553]);
ur5.links(1,3).I = [0.0025, 0, 0; 0, 0.0034, 0; 0, 0, 0.0546];
ur5.links(1,4).I = diag([0.0012,0.0012,0.0009]);
ur5.links(1,5).I = diag([0.0012,0.0012,0.0009]);
ur5.links(1,5).r = [0, 0.0018,0.01634];
ur5.links(1,6).I = diag([0.0001,0.0001,0.0001]);
ur5.links(1,6).r = [0, 0, -0.001159];

%% set motor inertia
ur5.links(1,1).Jm = 0*1.87*10^-8;
ur5.links(1,2).Jm = 0*1.87*10^-8;
ur5.links(1,3).Jm = 0*1.87*10^-8;
ur5.links(1,4).Jm = 0*1.87*10^-5;
ur5.links(1,5).Jm = 0*1.87*10^-5;
ur5.links(1,6).Jm = 0*1.87*10^-5;

%% saturation: https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques/
q1_limit_torque = 150;
q2_limit_torque = 150;
q3_limit_torque = 150;
q4_limit_torque = 28;
q5_limit_torque = 28;
q6_limit_torque = 28;

%%

Ts = 1/100;
init_config = [0,-37,-110,-2.5,-1,55]'*pi/180;
init_ee_pose = [0.1916, -0.1865, 0.4844];
