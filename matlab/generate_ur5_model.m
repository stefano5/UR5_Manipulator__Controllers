%% generate UR5 model. credit to: https://www.petercorke.com/RTB/r9/html/CodeGenerator.html
% mdl_ur5
% cg = CodeGenerator(ur5);
% cg.geneverything();   <- it could takes hours...
% instead of cg.geneverything() use: 
%       cg.gencoriolis();   <- on my pc it takes ~1hr
%       cg.genfkine();      
%       cg.gengravload(); 
%       cg.genjacobian();   
%       cg.geninertia();

%%
parametersUR5

addpath UR5
addpath UR5/symbolicexpressions/
addpath regressore

load('C_matrix_symb_m.mat') 

load('inertia_row_1.mat')
load('inertia_row_2.mat')
load('inertia_row_3.mat')
load('inertia_row_4.mat')
load('inertia_row_5.mat')
load('inertia_row_6.mat')

load('jacob0.mat')

load('gravload.mat')

%%
M = (vpa([inertia_row_1;inertia_row_2;inertia_row_3;inertia_row_4;inertia_row_5;inertia_row_6],5));
C = (vpa(C,5));
G = (vpa(gravload,5));
Jac = vpa(jacob0, 5);

%% store it as simulink block
if 1==0
    matlabFunction(M, 'File', 'matrix_simulink/Mmatrix');
    matlabFunction(C, 'File', 'matrix_simulink/Cmatrix');
    matlabFunction(G, 'File', 'matrix_simulink/Gvector');
    matlabFunction(Jac, 'File', 'matrix_simulink/Jacobian0');
end





