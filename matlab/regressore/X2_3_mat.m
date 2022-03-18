function X2_3 = X2_3_mat(q1,q2,q3,q1rdot,q2rdot,q3rdot)
%X2_3_MAT
%    X2_3 = X2_3_MAT(Q1,Q2,Q3,Q1RDOT,Q2RDOT,Q3RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:15

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = t2.^2;
t9 = t5.^2;
t10 = t3.*t4;
t11 = t3.*t7;
t12 = t4.*t6;
t13 = t6.*t7;
t14 = t5.*t13;
t15 = -t13;
t16 = t2.*t10;
t17 = t2.*t11;
t18 = t2.*t12;
t19 = t5.*t10;
t20 = t2.*t13;
t21 = t5.*t11;
t22 = t5.*t12;
t23 = t8+t9;
t26 = t11+t12;
t24 = -t19;
t25 = t2.*t15;
t27 = q1rdot.*t23;
t28 = t5.*t23;
t29 = q2rdot.*t2.*t23;
t30 = q3rdot.*t2.*t23;
t31 = t10+t15;
t32 = t17+t18;
t33 = t21+t22;
t34 = t16+t25;
t35 = t14+t24;
t36 = t5.*t32;
t37 = t2.*t33;
t41 = t29+t30;
t38 = t5.*t34;
t39 = t2.*t35;
t40 = -t37;
t42 = t36+t40;
t44 = t38+t39;
t43 = q1rdot.*t42;
t45 = q2rdot.*t2.*t42;
t46 = q3rdot.*t2.*t42;
t47 = q2rdot.*t5.*t42;
t48 = q3rdot.*t5.*t42;
t49 = q1rdot.*t44;
t50 = t2.*t44;
t51 = q2rdot.*t5.*t44;
t53 = q3rdot.*t5.*t44;
t52 = -t47;
t54 = -t48;
t55 = -t49;
t56 = t28+t50;
t59 = t51+t53;
t60 = t27+t45+t46;
t57 = q2rdot.*t56;
t58 = q3rdot.*t56;
t62 = t52+t54+t55;
t61 = t57+t58;
X2_3 = reshape([-q2rdot.*t5.*t26-q3rdot.*t5.*t26,t59,t59,0.0,0.0,0.0,-q2rdot.*t2.*t26-q3rdot.*t2.*t26,t61,t61,0.0,0.0,0.0,q1rdot.*t26-q2rdot.*t5.*t31-q3rdot.*t5.*t31,t62,t62,0.0,0.0,0.0,0.0,t41,t41,0.0,0.0,0.0,q2rdot.*t2.*t31+q3rdot.*t2.*t31,t60,t60,0.0,0.0,0.0,q1rdot.*t31,t43,t43,0.0,0.0,0.0],[6,6]);
