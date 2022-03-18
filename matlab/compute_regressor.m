%%
%Vettori posizione delle origini dei frame di ogni link rispetto a {S0}
syms q1 q2 q3 q4 q5 q6 q1dot q2dot q3dot q4dot q5dot q6dot real
syms q1rdot q2rdot q3rdot q4rdot q5rdot q6rdot real
q = [q1 q2 q3 q4 q5 q6]';
qdot = [q1dot q2dot q3dot q4dot q5dot q6dot]';
qrdot = [q1rdot q2rdot q3rdot q4rdot q5rdot q6rdot]';
rotz=@(q)[cos(q), -sin(q), 0;sin(q),  cos(q), 0;      0,        0, 1];
roty=@(q)[ cos(q), 0, sin(q);       0, 1,       0;-sin(q), 0, cos(q)];
rotx=@(q)[1,       0,        0; 0, cos(q), -sin(q);0, sin(q),  cos(q)];


%T1 = ur5.links(1,1).A(q);
R01 = rotz(q1);
R02 = R01*roty(q2);
R03 = R02*roty(q3);
R04 = R03*roty(q4);
R05 = R04*rotz(q5);
R06 = R05*roty(q6);
%%
print_var(R01, "toMathematica/R01");
print_var(R02, "toMathematica/R02");
print_var(R03, "toMathematica/R03");
print_var(R04, "toMathematica/R04");
print_var(R05, "toMathematica/R05");
print_var(R06, "toMathematica/R06");

%%

P01 = [0; 0; ur5.links(1,1).d];
P12 = R02*[ur5.links(1,2).a; 0; 0];
P23 = R03*[ur5.links(1,3).a; 0; 0];
P34 = R04*[0; -ur5.links(1,4).d; 0];
P45 = R05*[0; 0; -ur5.links(1,5).d];
P56 = R06*[0; -ur5.links(1,6).d; 0];

%%
%Jacobiano di posizione e orientazione dell'origine del frame
%di ogni link in terna {S0}
Jv1 = sym(zeros(3,6));
Jom1 = sym(zeros(3,6));
Jom1(3,1) = 1;
print_var(Jv1 , "toMathematica/Jv1");
print_var(Jom1 , "toMathematica/Jom1");

Jv2 = sym(zeros(3,6));
Jom2 = sym(zeros(3,6));
Jv2(:,1) = cross([0; 0; 1], P01 + P12);
Jv2(:,2) = cross(R01*[0; -1; 0], P12);
Jom2(:,1) = Jom1(:,1);
Jom2(:,2) = R01*[0; -1; 0];
print_var(Jv2 , "toMathematica/Jv2");
print_var(Jom2 , "toMathematica/Jom2");


Jv3 = sym(zeros(3,6));
Jom3 = sym(zeros(3,6));
Jv3(:,1) = cross([0; 0; 1], P01 + P12 + P23);
Jv3(:,2) = cross(R01*[0; -1; 0], P12 + P23);
Jv3(:,3) = cross(R02*[0; -1; 0], P23);
Jom3(:,1:2) = Jom2(:,1:2);
Jom3(:,3) = R02*[0; -1; 0];
print_var(Jv3 , "toMathematica/Jv3");
print_var(Jom3 , "toMathematica/Jom3");


Jv4 = sym(zeros(3,6));
Jom4 = sym(zeros(3,6));
Jv4(:,1) = cross([0; 0; 1], P01 + P12 + P23 + P34);
Jv4(:,2) = cross(R01*[0; -1; 0], P12 + P23 + P34);
Jv4(:,3) = cross(R02*[0; -1; 0], P23 + P34);
Jv4(:,4) = cross(R03*[0; -1; 0], P34);
Jom4(:,1:3) = Jom3(:,1:3);
Jom4(:,4) = R03*[0; -1; 0];
print_var(Jv4 , "toMathematica/Jv4");
print_var(Jom4 , "toMathematica/Jom4");


Jv5 = sym(zeros(3,6));
Jom5 = sym(zeros(3,6));
Jv5(:,1) = cross([0; 0; 1], P01 + P12 + P23 + P34 + P45);
Jv5(:,2) = cross(R01*[0; -1; 0], P12 + P23 + P34 + P45);
Jv5(:,3) = cross(R02*[0; -1; 0], P23 + P34 + P45);
Jv5(:,4) = cross(R03*[0; -1; 0], P34 + P45);
Jv5(:,5) = cross(R04*[0; 0; -1], P45);
Jom5(:,1:4) = Jom4(:,1:4);
Jom5(:,5) = R04*[0; 0; -1];
print_var(Jv5 , "toMathematica/Jv5");
print_var(Jom5 , "toMathematica/Jom5");


Jv6 = sym(zeros(3,6));
Jom6 = sym(zeros(3,6));
Jv6(:,1) = cross([0; 0; 1], P01 + P12 + P23 + P34 + P45 + P56);
Jv6(:,2) = cross(R01*[0; -1; 0], P12 + P23 + P34 + P45 + P56);
Jv6(:,3) = cross(R02*[0; -1; 0], P23 + P34 + P45 + P56);
Jv6(:,4) = cross(R03*[0; -1; 0], P34 + P45 + P56);
Jv6(:,5) = cross(R04*[0; 0; -1], P45 + P56);
Jv6(:,6) = cross(R05*[0; -1; 0], P56);
Jom6(:,1:5) = Jom5(:,1:5);
Jom6(:,6) = R05*[0; -1; 0];
print_var(Jv6, "toMathematica/Jv6");
print_var(Jom6, "toMathematica/Jom6");


%%
%Calcolo termini X, W e Z per il regressore Y
%Il secondo indice si riferisce al link, il primo al tipo di parametro
%dinamico
E1 = zeros(3);
E1(1,1) = 1;
E2 = zeros(3);
E2(1,2) = 1;
E2(2,1) = 1;
E3 = zeros(3);
E3(1,3) = 1;
E3(3,1) = 1;
E4 = zeros(3);
E4(2,2) = 1;
E5 = zeros(3);
E5(2,3) = 1;
E5(3,2) = 1;
E6 = zeros(3);
E6(3,3) = 1;

E = zeros(3,6,3);
E(:,1,:) = E1;
E(:,2,:) = -E2;
E(:,3,:) = -E3;
E(:,4,:) = E4;
E(:,5,:) = -E5;
E(:,6,:) = E6;

X0_1 = Jv1'*Jv1*qrdot;
X0_2 = Jv2'*Jv2*qrdot;
X0_3 = Jv3'*Jv3*qrdot;
X0_4 = Jv4'*Jv4*qrdot;
X0_5 = Jv5'*Jv5*qrdot;
X0_6 = Jv6'*Jv6*qrdot;

X1_1 = (Jv1'*skew(Jom1*qrdot) - Jom1'*skew(Jv1*qrdot))*R01;
X1_2 = (Jv2'*skew(Jom2*qrdot) - Jom2'*skew(Jv2*qrdot))*R02;
X1_3 = (Jv3'*skew(Jom3*qrdot) - Jom3'*skew(Jv3*qrdot))*R03;
X1_4 = (Jv4'*skew(Jom4*qrdot) - Jom4'*skew(Jv4*qrdot))*R04;
X1_5 = (Jv5'*skew(Jom5*qrdot) - Jom5'*skew(Jv5*qrdot))*R05;
X1_6 = (Jv6'*skew(Jom6*qrdot) - Jom6'*skew(Jv6*qrdot))*R06;

X2_1 = [Jom1'*R01*E1*Jom1*qrdot, Jom1'*R01*(-E2)*Jom1*qrdot, Jom1'*R01*(-E3)*Jom1*qrdot, Jom1'*R01*E4*Jom1*qrdot, Jom1'*R01*(-E5)*Jom1*qrdot, Jom1'*R01*E6*Jom1*qrdot];
X2_2 = [Jom2'*R02*E1*Jom2*qrdot, Jom2'*R02*(-E2)*Jom2*qrdot, Jom2'*R02*(-E3)*Jom2*qrdot, Jom2'*R02*E4*Jom2*qrdot, Jom2'*R02*(-E5)*Jom2*qrdot, Jom2'*R02*E6*Jom2*qrdot];
X2_3 = [Jom3'*R03*E1*Jom3*qrdot, Jom3'*R03*(-E2)*Jom3*qrdot, Jom3'*R03*(-E3)*Jom3*qrdot, Jom3'*R03*E4*Jom3*qrdot, Jom3'*R03*(-E5)*Jom3*qrdot, Jom3'*R03*E6*Jom3*qrdot];
X2_4 = [Jom4'*R04*E1*Jom4*qrdot, Jom4'*R04*(-E2)*Jom4*qrdot, Jom4'*R04*(-E3)*Jom4*qrdot, Jom4'*R04*E4*Jom4*qrdot, Jom4'*R04*(-E5)*Jom4*qrdot, Jom4'*R04*E6*Jom4*qrdot];
X2_5 = [Jom5'*R05*E1*Jom5*qrdot, Jom5'*R05*(-E2)*Jom5*qrdot, Jom5'*R05*(-E3)*Jom5*qrdot, Jom5'*R05*E4*Jom5*qrdot, Jom5'*R05*(-E5)*Jom5*qrdot, Jom5'*R05*E6*Jom5*qrdot];
X2_6 = [Jom6'*R06*E1*Jom6*qrdot, Jom6'*R06*(-E2)*Jom6*qrdot, Jom6'*R06*(-E3)*Jom6*qrdot, Jom6'*R06*E4*Jom6*qrdot, Jom6'*R06*(-E5)*Jom6*qrdot, Jom6'*R06*E6*Jom6*qrdot];

W0_1 = (1/2)*[qrdot'*diff(Jv1'*Jv1, q1)*qdot; qrdot'*diff(Jv1'*Jv1, q2)*qdot; qrdot'*diff(Jv1'*Jv1, q3)*qdot; qrdot'*diff(Jv1'*Jv1, q4)*qdot; qrdot'*diff(Jv1'*Jv1, q5)*qdot; qrdot'*diff(Jv1'*Jv1, q6)*qdot];
W0_2 = (1/2)*[qrdot'*diff(Jv2'*Jv2, q1)*qdot; qrdot'*diff(Jv2'*Jv2, q2)*qdot; qrdot'*diff(Jv2'*Jv2, q3)*qdot; qrdot'*diff(Jv2'*Jv2, q4)*qdot; qrdot'*diff(Jv2'*Jv2, q5)*qdot; qrdot'*diff(Jv2'*Jv2, q6)*qdot];
W0_3 = (1/2)*[qrdot'*diff(Jv3'*Jv3, q1)*qdot; qrdot'*diff(Jv3'*Jv3, q2)*qdot; qrdot'*diff(Jv3'*Jv3, q3)*qdot; qrdot'*diff(Jv3'*Jv3, q4)*qdot; qrdot'*diff(Jv3'*Jv3, q5)*qdot; qrdot'*diff(Jv3'*Jv3, q6)*qdot];
W0_4 = (1/2)*[qrdot'*diff(Jv4'*Jv4, q1)*qdot; qrdot'*diff(Jv4'*Jv4, q2)*qdot; qrdot'*diff(Jv4'*Jv4, q3)*qdot; qrdot'*diff(Jv4'*Jv4, q4)*qdot; qrdot'*diff(Jv4'*Jv4, q5)*qdot; qrdot'*diff(Jv4'*Jv4, q6)*qdot];
W0_5 = (1/2)*[qrdot'*diff(Jv5'*Jv5, q1)*qdot; qrdot'*diff(Jv5'*Jv5, q2)*qdot; qrdot'*diff(Jv5'*Jv5, q3)*qdot; qrdot'*diff(Jv5'*Jv5, q4)*qdot; qrdot'*diff(Jv5'*Jv5, q5)*qdot; qrdot'*diff(Jv5'*Jv5, q6)*qdot];
W0_6 = (1/2)*[qrdot'*diff(Jv6'*Jv6, q1)*qdot; qrdot'*diff(Jv6'*Jv6, q2)*qdot; qrdot'*diff(Jv6'*Jv6, q3)*qdot; qrdot'*diff(Jv6'*Jv6, q4)*qdot; qrdot'*diff(Jv6'*Jv6, q5)*qdot; qrdot'*diff(Jv6'*Jv6, q6)*qdot];

W1_1 = (1/2)*[diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q1),...
    diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q2),...
    diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q3),...
    diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q4),...
    diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q5),...
    diff(R01'*(skew(Jom1*qdot))'*Jv1*qrdot - R01'*(skew(Jv1*qdot))'*Jom1*qrdot, q6)]';
W1_2 = (1/2)*[diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q1),...
    diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q2),...
    diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q3),...
    diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q4),...
    diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q5),...
    diff(R02'*(skew(Jom2*qdot))'*Jv2*qrdot - R02'*(skew(Jv2*qdot))'*Jom2*qrdot, q6)]';
W1_3 = (1/2)*[diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q1),...
    diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q2),...
    diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q3),...
    diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q4),...
    diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q5),...
    diff(R03'*(skew(Jom3*qdot))'*Jv3*qrdot - R03'*(skew(Jv3*qdot))'*Jom3*qrdot, q6)]';
W1_4 = (1/2)*[diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q1),...
    diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q2),...
    diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q3),...
    diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q4),...
    diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q5),...
    diff(R04'*(skew(Jom4*qdot))'*Jv4*qrdot - R04'*(skew(Jv4*qdot))'*Jom4*qrdot, q6)]';
W1_5 = (1/2)*[diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q1),...
    diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q2),...
    diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q3),...
    diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q4),...
    diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q5),...
    diff(R05'*(skew(Jom5*qdot))'*Jv5*qrdot - R05'*(skew(Jv5*qdot))'*Jom5*qrdot, q6)]';
W1_6 = (1/2)*[diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q1),...
    diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q2),...
    diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q3),...
    diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q4),...
    diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q5),...
    diff(R06'*(skew(Jom6*qdot))'*Jv6*qrdot - R06'*(skew(Jv6*qdot))'*Jom6*qrdot, q6)]';

loadWi
% W2_1 = (1/2)*[jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q);...
%     jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q);...
%     jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q);...
%     jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q);...
%     jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q);...
%     jacobian(qrdot'*Jom1'*R01*E*R01'*Jom1*qdot, q)]';
% W2_2 = (1/2)*[qrdot'*diff(Jom2'*R02*E1*R02'*Jom2, q1)*qdot;...
%     qrdot'*diff(Jom2'*R02*(-E2)*R02'*Jom2, q2)*qdot;...
%     qrdot'*diff(Jom2'*R02*(-E3)*R02'*Jom2, q3)*qdot;...
%     qrdot'*diff(Jom2'*R02*E4*R02'*Jom2, q4)*qdot;...
%     qrdot'*diff(Jom2'*R02*(-E5)*R02'*Jom2, q5)*qdot;...
%     qrdot'*diff(Jom2'*R02*E6*R02'*Jom2, q6)*qdot];
% W2_3 = (1/2)*[qrdot'*diff(Jom3'*R03*E1*R03'*Jom3, q1)*qdot;...
%     qrdot'*diff(Jom3'*R03*(-E2)*R03'*Jom3, q2)*qdot;...
%     qrdot'*diff(Jom3'*R03*(-E3)*R03'*Jom3, q3)*qdot;...
%     qrdot'*diff(Jom3'*R03*E4*R03'*Jom3, q4)*qdot;...
%     qrdot'*diff(Jom3'*R03*(-E5)*R03'*Jom3, q5)*qdot;...
%     qrdot'*diff(Jom3'*R03*E6*R03'*Jom3, q6)*qdot];
% W2_4 = (1/2)*[qrdot'*diff(Jom4'*R04*E1*R04'*Jom4, q1)*qdot;...
%     qrdot'*diff(Jom4'*R04*(-E2)*R04'*Jom4, q2)*qdot;...
%     qrdot'*diff(Jom4'*R04*(-E3)*R04'*Jom4, q3)*qdot;...
%     qrdot'*diff(Jom4'*R04*E4*R04'*Jom4, q4)*qdot;...
%     qrdot'*diff(Jom4'*R04*(-E5)*R04'*Jom4, q5)*qdot;...
%     qrdot'*diff(Jom4'*R04*E6*R04'*Jom4, q6)*qdot];
% W2_5 = (1/2)*[qrdot'*diff(Jom5'*R05*E1*R05'*Jom5, q1)*qdot;...
%     qrdot'*diff(Jom5'*R05*(-E2)*R05'*Jom5, q2)*qdot;...
%     qrdot'*diff(Jom5'*R05*(-E3)*R05'*Jom5, q3)*qdot;...
%     qrdot'*diff(Jom5'*R05*E4*R05'*Jom5, q4)*qdot;...
%     qrdot'*diff(Jom5'*R05*(-E5)*R05'*Jom5, q5)*qdot;...
%     qrdot'*diff(Jom5'*R05*E6*R05'*Jom5, q6)*qdot];
% W2_6 = (1/2)*[qrdot'*diff(Jom6'*R06*E1*R06'*Jom6, q1)*qdot;...
%     qrdot'*diff(Jom6'*R06*(-E2)*R06'*Jom6, q2)*qdot;...
%     qrdot'*diff(Jom6'*R06*(-E3)*R06'*Jom6, q3)*qdot;...
%     qrdot'*diff(Jom6'*R06*E4*R06'*Jom6, q4)*qdot;...
%     qrdot'*diff(Jom6'*R06*(-E5)*R06'*Jom6, q5)*qdot;...
%     qrdot'*diff(Jom6'*R06*E6*R06'*Jom6, q6)*qdot];

g = [0; 0; -9.81];
Z0_1 = -Jv1'*g;
Z0_2 = -Jv2'*g;
Z0_3 = -Jv3'*g;
Z0_4 = -Jv4'*g;
Z0_5 = -Jv5'*g;
Z0_6 = -Jv6'*g;

Z1_1 = -[(diff(R01'*g, q1))'; (diff(R01'*g, q2))'; (diff(R01'*g, q3))'; (diff(R01'*g, q4))'; (diff(R01'*g, q5))'; (diff(R01'*g, q6))'];
Z1_2 = -[(diff(R02'*g, q1))'; (diff(R02'*g, q2))'; (diff(R02'*g, q3))'; (diff(R02'*g, q4))'; (diff(R02'*g, q5))'; (diff(R02'*g, q6))'];
Z1_3 = -[(diff(R03'*g, q1))'; (diff(R03'*g, q2))'; (diff(R03'*g, q3))'; (diff(R03'*g, q4))'; (diff(R03'*g, q5))'; (diff(R03'*g, q6))'];
Z1_4 = -[(diff(R04'*g, q1))'; (diff(R04'*g, q2))'; (diff(R04'*g, q3))'; (diff(R04'*g, q4))'; (diff(R04'*g, q5))'; (diff(R04'*g, q6))'];
Z1_5 = -[(diff(R05'*g, q1))'; (diff(R05'*g, q2))'; (diff(R05'*g, q3))'; (diff(R05'*g, q4))'; (diff(R05'*g, q5))'; (diff(R05'*g, q6))'];
Z1_6 = -[(diff(R06'*g, q1))'; (diff(R06'*g, q2))'; (diff(R06'*g, q3))'; (diff(R06'*g, q4))'; (diff(R06'*g, q5))'; (diff(R06'*g, q6))'];

%%
%Save matlab functions

if 1==0
    matlabFunction(X0_1, 'File', 'regressore/X0_1_mat.m');
    matlabFunction(X0_2, 'File', 'regressore/X0_2_mat.m');
    matlabFunction(X0_3, 'File', 'regressore/X0_3_mat.m');
    matlabFunction(X0_4, 'File', 'regressore/X0_4_mat.m');
    matlabFunction(X0_5, 'File', 'regressore/X0_5_mat.m');
    matlabFunction(X0_6, 'File', 'regressore/X0_6_mat.m');

    matlabFunction(X1_1, 'File', 'regressore/X1_1_mat.m');
    matlabFunction(X1_2, 'File', 'regressore/X1_2_mat.m');
    matlabFunction(X1_3, 'File', 'regressore/X1_3_mat.m');
    matlabFunction(X1_4, 'File', 'regressore/X1_4_mat.m');
    matlabFunction(X1_5, 'File', 'regressore/X1_5_mat.m');
    matlabFunction(X1_6, 'File', 'regressore/X1_6_mat.m');

    matlabFunction(X2_1, 'File', 'regressore/X2_1_mat.m');
    matlabFunction(X2_2, 'File', 'regressore/X2_2_mat.m');
    matlabFunction(X2_3, 'File', 'regressore/X2_3_mat.m');
    matlabFunction(X2_4, 'File', 'regressore/X2_4_mat.m');
    matlabFunction(X2_5, 'File', 'regressore/X2_5_mat.m');
    matlabFunction(X2_6, 'File', 'regressore/X2_6_mat.m');

    matlabFunction(W0_1, 'File', 'regressore/W0_1_mat.m');
    matlabFunction(W0_2, 'File', 'regressore/W0_2_mat.m');
    matlabFunction(W0_3, 'File', 'regressore/W0_3_mat.m');
    matlabFunction(W0_4, 'File', 'regressore/W0_4_mat.m');
    matlabFunction(W0_5, 'File', 'regressore/W0_5_mat.m');
    matlabFunction(W0_6, 'File', 'regressore/W0_6_mat.m');

    matlabFunction(W1_1, 'File', 'regressore/W1_1_mat.m');
    matlabFunction(W1_2, 'File', 'regressore/W1_2_mat.m');
    matlabFunction(W1_3, 'File', 'regressore/W1_3_mat.m');
    matlabFunction(W1_4, 'File', 'regressore/W1_4_mat.m');
    matlabFunction(W1_5, 'File', 'regressore/W1_5_mat.m');
    matlabFunction(W1_6, 'File', 'regressore/W1_6_mat.m');

    matlabFunction(Z0_1, 'File', 'regressore/Z0_1_mat.m');
    matlabFunction(Z0_2, 'File', 'regressore/Z0_2_mat.m');
    matlabFunction(Z0_3, 'File', 'regressore/Z0_3_mat.m');
    matlabFunction(Z0_4, 'File', 'regressore/Z0_4_mat.m');
    matlabFunction(Z0_5, 'File', 'regressore/Z0_5_mat.m');
    matlabFunction(Z0_6, 'File', 'regressore/Z0_6_mat.m');

    matlabFunction(Z1_1, 'File', 'regressore/Z1_1_mat.m');
    matlabFunction(Z1_2, 'File', 'regressore/Z1_2_mat.m');
    matlabFunction(Z1_3, 'File', 'regressore/Z1_3_mat.m');
    matlabFunction(Z1_4, 'File', 'regressore/Z1_4_mat.m');
    matlabFunction(Z1_5, 'File', 'regressore/Z1_5_mat.m');
    matlabFunction(Z1_6, 'File', 'regressore/Z1_6_mat.m');

end


