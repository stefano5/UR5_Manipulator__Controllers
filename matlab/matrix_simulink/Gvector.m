function G = Gvector(m2,m3,m4,m5,m6,q2,q3,q4,q5)
%GVECTOR
%    G = GVECTOR(M2,M3,M4,M5,M6,Q2,Q3,Q4,Q5)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    30-Jan-2022 15:26:29

t2 = cos(q2);
t3 = cos(q4);
t4 = cos(q5);
t5 = sin(q5);
t6 = q2.*2.0;
t7 = q3.*2.0;
t8 = q4.*2.0;
t9 = q5.*-1.0;
t10 = q5.*1.0;
t11 = q5.*-2.0;
t12 = q5.*2.0;
t13 = q2+q3;
t14 = q3+q4;
t15 = cos(t13);
t16 = cos(t14);
t17 = q4+t13;
t21 = q2+t11;
t22 = q2+t12;
t23 = q4+t11;
t24 = q4+t12;
t25 = m5.*t3.*3.404815978676077e-39;
t26 = m6.*t3.*3.404815978676077e-39;
t31 = t8+t13;
t32 = t6+t14;
t33 = t11+t13;
t34 = t12+t13;
t35 = t11+t14;
t36 = t12+t14;
t61 = q2+t7+t8;
t62 = q4+t6+t7;
t79 = t6+t7+t8;
t18 = cos(t17);
t19 = q5+t17;
t20 = sin(t17);
t27 = cos(t21);
t28 = cos(t22);
t29 = cos(t23);
t30 = cos(t24);
t38 = cos(t31);
t39 = cos(t32);
t40 = cos(t33);
t41 = cos(t34);
t42 = cos(t35);
t43 = cos(t36);
t44 = m3.*t15.*-2.376472500000091;
t45 = m3.*t15.*2.376472500000091;
t46 = t9+t17;
t47 = t11+t17;
t48 = t12+t17;
t49 = m4.*t15.*-3.847972500000196;
t50 = m4.*t15.*3.847972500000196;
t51 = m5.*t15.*-3.847972500000196;
t52 = m5.*t15.*3.847972500000196;
t53 = m6.*t15.*-3.847972500000196;
t54 = m6.*t15.*3.847972500000196;
t63 = cos(t61);
t64 = cos(t62);
t65 = t11+t31;
t66 = t12+t31;
t67 = t11+t32;
t68 = t12+t32;
t92 = t7+t8+t21;
t93 = t7+t8+t22;
t94 = t6+t7+t23;
t95 = t6+t7+t24;
t96 = sin(t79);
t103 = t11+t79;
t104 = t12+t79;
t37 = sin(t19);
t55 = sin(t46);
t56 = sin(t47);
t57 = sin(t48);
t58 = m4.*t20.*1.602953999963859e-1;
t59 = m5.*t20.*9.108584999671621e-1;
t60 = m6.*t20.*9.285164998367463e-1;
t69 = m5.*t30.*8.300241499188834e-30;
t70 = m5.*t29.*-8.300241502593922e-30;
t71 = m5.*t29.*8.300241502593922e-30;
t72 = m6.*t30.*8.300241499188834e-30;
t73 = m6.*t29.*-8.300241502593922e-30;
t74 = m6.*t29.*8.300241502593922e-30;
t80 = cos(t65);
t81 = cos(t66);
t82 = cos(t67);
t83 = cos(t68);
t84 = m5.*t38.*-4.046856535248515e-20;
t85 = m5.*t38.*4.046856535248515e-20;
t86 = m5.*t40.*2.023428267624257e-20;
t87 = m5.*t41.*2.023428267624257e-20;
t88 = m6.*t38.*-4.046856535248515e-20;
t89 = m6.*t38.*4.046856535248515e-20;
t90 = m6.*t40.*2.023428267624257e-20;
t91 = m6.*t41.*2.023428267624257e-20;
t99 = cos(t92);
t100 = cos(t93);
t101 = cos(t94);
t102 = cos(t95);
t113 = sin(t103);
t114 = sin(t104);
t75 = m5.*t37.*-8.014769998175097e-2;
t76 = m5.*t37.*8.014769998175097e-2;
t77 = m6.*t37.*-3.979966049183759e-1;
t78 = m6.*t37.*3.979966049183759e-1;
t97 = m5.*t55.*8.014770001462068e-2;
t98 = m6.*t55.*3.979966050816302e-1;
t105 = m5.*t81.*-2.023428266794105e-20;
t106 = m5.*t81.*2.023428266794105e-20;
t107 = m5.*t80.*-2.023428268454102e-20;
t108 = m5.*t80.*2.023428268454102e-20;
t109 = m6.*t81.*-2.023428266794105e-20;
t110 = m6.*t81.*2.023428266794105e-20;
t111 = m6.*t80.*-2.023428268454102e-20;
t112 = m6.*t80.*2.023428268454102e-20;
G = [m4.*t2.*-1.798650165903859e-29-m5.*t2.*8.993250829519297e-30+m5.*t3.*8.30024150089066e-30-m6.*t2.*8.993250829519297e-30+m6.*t3.*8.30024150089066e-30+m5.*t15.*8.30024150089066e-30+m5.*t16.*8.993250829519297e-30+m6.*t15.*8.30024150089066e-30+m6.*t16.*8.993250829519297e-30+m5.*t20.*8.561048224485782e-66+m6.*t20.*8.561048224485782e-66-m5.*t27.*1.348987624427679e-29-m5.*t28.*1.348987624427679e-29-m6.*t27.*1.348987624427679e-29-m5.*t29.*2.023428268039334e-20-m6.*t28.*1.348987624427679e-29+m5.*t30.*2.023428267209181e-20-m6.*t29.*2.023428268039334e-20+m6.*t30.*2.023428267209181e-20+m5.*t38.*8.30024150089066e-30+m5.*t39.*8.993250829519297e-30+m6.*t38.*8.30024150089066e-30-m5.*t40.*1.245036225133599e-29+m6.*t39.*8.993250829519297e-30-m5.*t41.*1.245036225133599e-29-m6.*t40.*1.245036225133599e-29-m5.*t42.*2.192369697684388e-20-m6.*t41.*1.245036225133599e-29+m5.*t43.*2.192369696784902e-20-m6.*t42.*2.192369697684388e-20+m6.*t43.*2.192369696784902e-20-m5.*t56.*2.087007586419185e-56+m5.*t57.*2.087007585563285e-56-m6.*t56.*2.087007586419185e-56+m6.*t57.*2.087007585563285e-56+m4.*t63.*1.798650165903859e-29+m5.*t63.*2.697975248855359e-29+m5.*t64.*8.30024150089066e-30+m6.*t63.*2.697975248855359e-29+m6.*t64.*8.30024150089066e-30+m5.*t80.*4.150120752147874e-30+m5.*t81.*4.150120748743503e-30+m6.*t80.*4.150120752147874e-30-m5.*t82.*2.192369697684388e-20+m6.*t81.*4.150120748743503e-30+m5.*t83.*2.192369696784902e-20-m6.*t82.*2.192369697684388e-20+m6.*t83.*2.192369696784902e-20-m4.*t96.*2.03507820527272e-46-m5.*t96.*3.052617307909079e-46-m6.*t96.*3.052617307909079e-46+m5.*t99.*4.49662541660425e-30+m5.*t100.*4.496625412915046e-30+m6.*t99.*4.49662541660425e-30-m5.*t101.*2.023428268039334e-20+m6.*t100.*4.496625412915046e-30+m5.*t102.*2.023428267209181e-20-m6.*t101.*2.023428268039334e-20+m6.*t102.*2.023428267209181e-20-m5.*t113.*5.087695515268749e-47-m5.*t114.*5.087695511094849e-47-m6.*t113.*5.087695515268749e-47-m6.*t114.*5.087695511094849e-47,t25+t26+t44+t49+t51+t53+t58+t59+t60+t69+t70+t72+t73+t75+t77+t84+t86+t87+t88+t90+t91+t97+t98+t105+t107+t109+t111-m2.*t2.*2.08462499999996-m3.*t2.*4.16924999999992-m4.*t2.*4.16924999999992-m5.*t2.*4.16924999999992-m6.*t2.*4.16924999999992+m5.*t16.*3.689093157265238e-39+m6.*t16.*3.689093157265238e-39+m5.*t27.*2.192369697234491e-20+m5.*t28.*2.192369697234491e-20+m6.*t27.*2.192369697234491e-20+m6.*t28.*2.192369697234491e-20-m5.*t42.*8.993250831363181e-30+m5.*t43.*8.993250827673977e-30-m6.*t42.*8.993250831363181e-30+m6.*t43.*8.993250827673977e-30-m4.*t63.*8.769478788937965e-20-m5.*t63.*1.31542181834088e-19-m6.*t63.*1.31542181834088e-19-m5.*t99.*2.192369698133978e-20-m5.*t100.*2.192369696335313e-20-m6.*t99.*2.192369698133978e-20-m6.*t100.*2.192369696335313e-20,t25+t26+t44+t49+t51+t53+t58+t59+t60+t69+t70+t72+t73+t75+t77+t84+t86+t87+t88+t90+t91+t97+t98+t105+t107+t109+t111,t58+t59+t60-m5.*t5.*t18.*1.602953999963859e-1+m5.*t4.*t20.*3.287713611449037e-11-m6.*t5.*t18.*7.95993210000006e-1+m6.*t4.*t20.*1.632609364459202e-10,(m5.*6.321231325457759e+28+m6.*3.138990397739076e+29).*(t5.*7.93455457355776e+15+t5.*t18.*7.93455457355776e+15-t4.*t20.*3.868562622766813e+25).*6.55495545150136e-56,0.0];
