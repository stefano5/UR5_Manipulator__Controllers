function X0_3 = X0_3_mat(q1,q2,q3,q1rdot,q2rdot,q3rdot)
%X0_3_MAT
%    X0_3 = X0_3_MAT(Q1,Q2,Q3,Q1RDOT,Q2RDOT,Q3RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:03

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = t2.^2;
t9 = t5.^2;
t10 = t6.*(1.7e+1./4.0e+1);
t11 = t2.*t3.*(1.7e+1./4.0e+1);
t12 = t3.*t5.*(1.7e+1./4.0e+1);
t13 = t3.*t7.*3.9225e-1;
t14 = t4.*t6.*3.9225e-1;
t15 = t5.*t6.*t7.*3.9225e-1;
t16 = t2.*t3.*t4.*3.9225e-1;
t17 = t3.*t4.*t5.*3.9225e-1;
t18 = t2.*t6.*t7.*3.9225e-1;
t19 = -t15;
t20 = -t17;
t21 = -t18;
t22 = t13+t14;
t23 = t22.^2;
t24 = t16+t21;
t25 = t15+t20;
t28 = t10+t22;
t32 = t12+t17+t19;
t26 = t2.*t24;
t27 = t5.*t25;
t29 = t28.^2;
t31 = t11+t24;
t34 = t5.*t32;
t35 = t9.*t22.*t28;
t36 = t8.*t22.*t28;
t38 = t2.*t22.*t32;
t42 = t2.*t28.*t32;
t30 = -t27;
t33 = t2.*t31;
t37 = t5.*t22.*t31;
t40 = -t38;
t41 = t5.*t28.*t31;
t43 = -t42;
t39 = t26+t30;
t44 = t33+t34;
t45 = t37+t40;
t46 = t41+t43;
t47 = t39.*t44;
t48 = t35+t36+t47;
X0_3 = [q2rdot.*t46+q3rdot.*t45+q1rdot.*(t31.^2+t32.^2);q1rdot.*t46+q3rdot.*t48+q2rdot.*(t8.*t29+t9.*t29+t44.^2);q1rdot.*t45+q2rdot.*t48+q3rdot.*(t8.*t23+t9.*t23+t39.^2);0.0;0.0;0.0];