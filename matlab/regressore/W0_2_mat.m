function W0_2 = W0_2_mat(q1,q2,q1dot,q2dot,q1rdot,q2rdot)
%W0_2_MAT
%    W0_2 = W0_2_MAT(Q1,Q2,Q1DOT,Q2DOT,Q1RDOT,Q2RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:19

t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = t2.^2;
t7 = t4.^2;
t8 = t3.*t5.*t6.*(2.89e+2./8.0e+2);
t9 = t3.*t5.*t7.*(2.89e+2./8.0e+2);
W0_2 = [0.0;q1dot.*q1rdot.*(t8+t9).*(-1.0./2.0)+(q2dot.*q2rdot.*(t8+t9-(t3.*t6.*(1.7e+1./4.0e+1)+t3.*t7.*(1.7e+1./4.0e+1)).*(t5.*t6.*(1.7e+1./4.0e+1)+t5.*t7.*(1.7e+1./4.0e+1)).*2.0))./2.0;0.0;0.0;0.0;0.0];
