function Z0_6 = Z0_6_mat(q1,q2,q3,q4,q5)
%Z0_6_MAT
%    Z0_6 = Z0_6_MAT(Q1,Q2,Q3,Q4,Q5)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:12:23

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = sin(q1);
t8 = sin(q2);
t9 = sin(q3);
t10 = sin(q4);
t11 = sin(q5);
t12 = t7.*t8.*t9;
t13 = t2.*t3.*t4;
t14 = t2.*t3.*t9;
t15 = t2.*t4.*t8;
t16 = t3.*t4.*t7;
t17 = t2.*t8.*t9;
t18 = t3.*t7.*t9;
t19 = t4.*t7.*t8;
t22 = t2.*1.0915e-1;
t23 = t7.*1.0915e-1;
t24 = t2.*t6.*8.23e-2;
t25 = t6.*t7.*8.23e-2;
t20 = -t16;
t21 = -t17;
t26 = t14+t15;
t27 = t18+t19;
t28 = t12.*3.9225e-1;
t29 = t13.*3.9225e-1;
t30 = t16.*3.9225e-1;
t31 = t17.*3.9225e-1;
t32 = -t28;
t33 = t13+t21;
t34 = t12+t20;
t35 = -t29;
t36 = t10.*t26;
t37 = t10.*t27;
t41 = t5.*t26.*9.465e-2;
t42 = t5.*t27.*9.465e-2;
t38 = t5.*t33;
t39 = -t36;
t40 = t5.*t34;
t43 = -t41;
t44 = t10.*t33.*9.465e-2;
t45 = t10.*t34.*9.465e-2;
t46 = -t44;
t47 = -t45;
t48 = t37+t40;
t49 = t38+t39;
t51 = t11.*(t36-t38).*(-8.23e-2);
t50 = t11.*t48.*8.23e-2;
Z0_6 = [0.0;t7.*(t22+t24+t30+t32+t42+t47+t50+t3.*t7.*(1.7e+1./4.0e+1)).*(-9.81e+2./1.0e+2)-t2.*(-t23-t25+t29-t31+t41+t44+t2.*t3.*(1.7e+1./4.0e+1)+t11.*(t36-t38).*8.23e-2).*(9.81e+2./1.0e+2);t7.*(t22+t24+t30+t32+t42+t47+t50).*(-9.81e+2./1.0e+2)-t2.*(-t23-t25+t29-t31+t41+t44+t11.*(t36-t38).*8.23e-2).*(9.81e+2./1.0e+2);t2.*(-t23-t25+t41+t44+t11.*(t36-t38).*8.23e-2).*(-9.81e+2./1.0e+2)-t7.*(t22+t24+t42+t47+t50).*(9.81e+2./1.0e+2);(t5.*t26+t10.*t33).*(t24+t42+t47+t50).*(9.81e+2./1.0e+2)-(t5.*t27-t10.*t34).*(-t25+t41+t44+t11.*(t36-t38).*8.23e-2).*(9.81e+2./1.0e+2);(t25+t51).*(t2.*t6+t11.*t48).*(9.81e+2./1.0e+2)-(t6.*t7-t11.*(t36-t38)).*(t24+t50).*(9.81e+2./1.0e+2)];
