function X0_2 = X0_2_mat(q1,q2,q1rdot,q2rdot)
%X0_2_MAT
%    X0_2 = X0_2_MAT(Q1,Q2,Q1RDOT,Q2RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:03

t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = t2.^2;
t7 = t3.^2;
t8 = t4.^2;
t9 = t5.^2;
X0_2 = [q1rdot.*(t6.*t7.*1.80625e-1+t7.*t8.*1.80625e-1);q2rdot.*(t6.*t9.*1.80625e-1+t8.*t9.*1.80625e-1+(t3.*t6.*(1.7e+1./4.0e+1)+t3.*t8.*(1.7e+1./4.0e+1)).^2);0.0;0.0;0.0;0.0];
