function Z0_4 = Z0_4_mat(q1,q2,q3)
%Z0_4_MAT
%    Z0_4 = Z0_4_MAT(Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:12:23

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = t2.*1.0915e-1;
t9 = t5.*1.0915e-1;
t10 = t5.*t6.*t7.*3.9225e-1;
t11 = t2.*t3.*t4.*3.9225e-1;
t12 = t3.*t4.*t5.*3.9225e-1;
t13 = t2.*t6.*t7.*3.9225e-1;
t14 = -t10;
t15 = -t11;
Z0_4 = [0.0;t2.*(t9+t13+t15-t2.*t3.*(1.7e+1./4.0e+1)).*(9.81e+2./1.0e+2)-t5.*(t8+t12+t14+t3.*t5.*(1.7e+1./4.0e+1)).*(9.81e+2./1.0e+2);t2.*(t9+t13+t15).*(9.81e+2./1.0e+2)-t5.*(t8+t12+t14).*(9.81e+2./1.0e+2);0.0;0.0;0.0];