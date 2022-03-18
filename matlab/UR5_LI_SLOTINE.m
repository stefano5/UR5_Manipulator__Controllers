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

%ur5.links(1,3).I = [0.0025, 0, 0.0034; 0, 0.0034, 0; 0.0034, 0, 0.0546];
ur5.links(1,3).I = [0.0025, 0, 0; 0, 0.0034, 0; 0, 0, 0.0546];


ur5.links(1,4).I = diag([0.0012,0.0012,0.0009]);

ur5.links(1,5).I = diag([0.0012,0.0012,0.0009]);
ur5.links(1,5).r = [0, 0.0018,0.01634];

ur5.links(1,6).I = diag([0.0001,0.0001,0.0001]);
ur5.links(1,6).r = [0, 0, -0.001159];

%% set motor inertia
ur5.links(1,1).Jm= 0*1.87*10^-8;
ur5.links(1,2).Jm= 0*1.87*10^-8;
ur5.links(1,3).Jm= 0*1.87*10^-8;
ur5.links(1,4).Jm= 0*1.87*10^-5;
ur5.links(1,5).Jm= 0*1.87*10^-5;
ur5.links(1,6).Jm= 0*1.87*10^-5;


%% saturation: https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques/
q1_limit_torque = 150;
q2_limit_torque = 150;
q3_limit_torque = 150;
q4_limit_torque = 28;
q5_limit_torque = 28;
q6_limit_torque = 28;

%%

Ts = 1/100;


%% posizionamento sulla traiettoria da inseguire
init_config = [0,-55.5,-131,-2.5,20,105]'*pi/180;

init_ee_pose = [0.1916, -0.1865, 0.4844];
final_ee_pose = [-0.0597   -0.1914    0.6397];

waypoints = [init_ee_pose(1), final_ee_pose(1);
        init_ee_pose(2), final_ee_pose(2);
        init_ee_pose(3), final_ee_pose(3)];
timepoints = [ 0, 6];

velocitybc = [(final_ee_pose(1)-init_ee_pose(1)), 0;
    0, 0;
    (final_ee_pose(3)-init_ee_pose(3)), 0]*0.001;
              
%%

%calcolo_jacobiani_e_regressore;

rotz=@(q)[cos(q), -sin(q), 0;sin(q),  cos(q), 0;      0,        0, 1];
roty=@(q)[ cos(q), 0, sin(q);       0, 1,       0;-sin(q), 0, cos(q)];
rotx=@(q)[1,       0,        0; 0, cos(q), -sin(q);0, sin(q),  cos(q)];

%%
init_cond_regress = zeros(60, 1);
init_cond_regress(1) = m1;
init_cond_regress(2) = m2;
init_cond_regress(3) = m3;
init_cond_regress(4) = m4;
init_cond_regress(5) = m5;
init_cond_regress(6) = m6;

P0G1 = [0;0;ur5.links(1,1).d] + rotx(-pi/2)*ur5.links(1,1).r';
P1G2 = [ur5.links(1,2).a;0;0] + ur5.links(1,2).r';
P2G3 = [ur5.links(1,3).a;0;0] + ur5.links(1,3).r';
P3G4 = [0;0;ur5.links(1,4).d] + rotx(-pi/2)*ur5.links(1,4).r';
P4G5 = [0;0;ur5.links(1,5).d] + rotx(pi/2)*ur5.links(1,5).r';
P5G6 = [0;0;ur5.links(1,6).d] + ur5.links(1,6).r';

init_cond_regress(7:9) = m1*P0G1;
init_cond_regress(10:12) = m2*P1G2;
init_cond_regress(13:15) = m3*P2G3;
init_cond_regress(16:18) = m4*P3G4;
init_cond_regress(19:21) = m5*P4G5;
init_cond_regress(22:24) = m6*P5G6;
par = 0.1;


for i = 1: 6
    init_cond_regress(25 + (i-1) * 6 ) = ur5.links(1,i).I(1,1);
    init_cond_regress(26 + (i-1) * 6 ) = ur5.links(1,i).I(1,2);
    init_cond_regress(27 + (i-1) * 6 ) = ur5.links(1,i).I(1,3);
    init_cond_regress(28 + (i-1) * 6 ) = ur5.links(1,i).I(2,2);
    init_cond_regress(29 + (i-1) * 6 ) = ur5.links(1,i).I(2,3);
    init_cond_regress(30 + (i-1) * 6 ) = ur5.links(1,i).I(3,3);
end

          
return
%% generate model: https://www.petercorke.com/RTB/r9/html/CodeGenerator.html

syms m1 m2 m3 m4 m5 m6

ur5.links(1,1).m=m1;
ur5.links(1,2).m=m2;
ur5.links(1,3).m=m3;
ur5.links(1,4).m=m4;
ur5.links(1,5).m=m5;
ur5.links(1,6).m=m6;


cg = CodeGenerator(ur5);
% % cg.gencoriolis()
%cg.geneverything(); it takes long time
