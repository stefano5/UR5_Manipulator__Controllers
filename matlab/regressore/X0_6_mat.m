function X0_6 = X0_6_mat(q1,q2,q3,q4,q5,q1rdot,q2rdot,q3rdot,q4rdot,q5rdot,q6rdot)
%X0_6_MAT
%    X0_6 = X0_6_MAT(Q1,Q2,Q3,Q4,Q5,Q1RDOT,Q2RDOT,Q3RDOT,Q4RDOT,Q5RDOT,Q6RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:06

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
t12 = t2.^2;
t13 = t7.^2;
t14 = t3.*t4;
t15 = t2.*t6;
t16 = t3.*t9;
t17 = t4.*t8;
t18 = t6.*t7;
t19 = t8.*t9;
t29 = t8.*(1.7e+1./4.0e+1);
t32 = t2.*t3.*(1.7e+1./4.0e+1);
t33 = t3.*t7.*(1.7e+1./4.0e+1);
t36 = t2.*1.0915e-1;
t37 = t7.*1.0915e-1;
t20 = t7.*t19;
t21 = -t19;
t22 = t2.*t14;
t23 = t2.*t16;
t24 = t2.*t17;
t25 = t7.*t14;
t26 = t2.*t19;
t27 = t7.*t16;
t28 = t7.*t17;
t34 = -t32;
t35 = t16+t17;
t39 = t16.*3.9225e-1;
t40 = t17.*3.9225e-1;
t43 = t15.*8.23e-2;
t44 = t18.*8.23e-2;
t30 = -t25;
t31 = t2.*t21;
t38 = t14+t21;
t41 = t5.*t35;
t42 = t10.*t35;
t45 = t23+t24;
t46 = t27+t28;
t47 = t20.*3.9225e-1;
t51 = t22.*3.9225e-1;
t52 = t25.*3.9225e-1;
t53 = t26.*3.9225e-1;
t48 = t5.*t38;
t49 = t10.*t38;
t50 = -t42;
t54 = -t47;
t55 = t22+t31;
t56 = t20+t30;
t57 = -t51;
t58 = t5.*t45;
t59 = t10.*t45;
t60 = t5.*t46;
t61 = t10.*t46;
t68 = t42.*9.465e-2;
t62 = t5.*t55;
t63 = t10.*t55;
t64 = -t59;
t65 = t5.*t56;
t66 = t10.*t56;
t69 = -t68;
t70 = t48.*9.465e-2;
t72 = t58.*9.465e-2;
t73 = t60.*9.465e-2;
t79 = t41+t49;
t80 = t48+t50;
t67 = -t66;
t71 = -t70;
t74 = -t72;
t75 = t63.*9.465e-2;
t76 = t66.*9.465e-2;
t81 = t58+t63;
t82 = t61+t65;
t83 = t62+t64;
t86 = t11.*t79.*8.23e-2;
t87 = -t11.*(t59-t62);
t92 = t11.*(t59-t62).*(-8.23e-2);
t77 = -t75;
t78 = -t76;
t84 = t60+t67;
t85 = t11.*t82;
t88 = -t86;
t90 = t18+t87;
t94 = t44+t92;
t97 = t69+t70+t86;
t118 = -t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2);
t121 = (t42-t48).*(-t44+t72+t75+t11.*(t59-t62).*8.23e-2);
t128 = -t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t133 = -t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t89 = t15+t85;
t91 = t85.*8.23e-2;
t96 = t86.*t90;
t98 = t97.^2;
t100 = t11.*t79.*t94;
t106 = t39+t40+t68+t71+t88;
t109 = t74+t77+t94;
t112 = t81.*t97;
t113 = t84.*t97;
t123 = -t84.*(-t44+t72+t75+t11.*(t59-t62).*8.23e-2);
t144 = -t7.*t97.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t93 = t43+t91;
t95 = t86.*t89;
t102 = -t100;
t104 = t89.*t94;
t108 = t106.^2;
t110 = t29+t106;
t115 = t37+t109;
t136 = t12.*t97.*t106;
t137 = t13.*t97.*t106;
t148 = t112+t121;
t151 = -t7.*t106.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t99 = t11.*t79.*t93;
t103 = t90.*t93;
t105 = -t104;
t107 = t73+t78+t93;
t111 = t110.^2;
t125 = t53+t57+t115;
t135 = t96+t102;
t138 = t12.*t97.*t110;
t139 = t13.*t97.*t110;
t141 = t12.*t106.*t110;
t142 = t13.*t106.*t110;
t153 = -t7.*t110.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t163 = t7.*t97.*t148;
t166 = t7.*t106.*t148;
t169 = t7.*t110.*t148;
t173 = -t148.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t174 = t148.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t101 = -t99;
t114 = t36+t107;
t119 = -t107.*(t42-t48);
t120 = t107.*(t42-t48);
t122 = t81.*t107;
t131 = t34+t125;
t140 = t103+t105;
t147 = t7.*t97.*t135;
t155 = t7.*t106.*t135;
t157 = t7.*t110.*t135;
t162 = -t135.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2);
t165 = -t163;
t177 = t135.*t148;
t116 = t7.*t114;
t124 = t52+t54+t114;
t134 = t95+t101;
t149 = t113+t120;
t158 = t122+t123;
t117 = -t116;
t126 = t7.*t124;
t129 = t33+t124;
t146 = t2.*t97.*t134;
t154 = t2.*t106.*t134;
t156 = t2.*t110.*t134;
t164 = t2.*t97.*t149;
t167 = t2.*t106.*t149;
t170 = t2.*t110.*t149;
t175 = t134.*t149;
t178 = -t140.*(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2));
t179 = t140.*(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2));
t181 = t140.*t158;
t185 = -t158.*(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2));
t127 = -t126;
t130 = t7.*t129;
t143 = t2.*t97.*t129;
t145 = t117+t118;
t150 = t2.*t106.*t129;
t152 = t2.*t110.*t129;
t161 = t129.*t134;
t168 = -t167;
t171 = -t170;
t172 = t129.*t149;
t176 = -t175;
t182 = -t140.*(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t187 = (t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2)).*(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t191 = -t158.*(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t197 = t146+t147+t179;
t202 = t164+t165+t185;
t132 = -t130;
t159 = t127+t128;
t180 = t143+t144;
t183 = -t140.*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t184 = t150+t151;
t186 = t152+t153;
t188 = -t187;
t189 = (t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2)).*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t192 = -t158.*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t193 = (t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)).*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2));
t194 = t161+t162;
t195 = t172+t174;
t199 = t154+t155+t182;
t203 = t166+t168+t191;
t204 = t176+t177+t181;
t160 = t132+t133;
t190 = -t189;
t196 = t136+t137+t188;
t200 = t141+t142+t193;
t201 = t156+t157+t183;
t205 = t169+t171+t192;
t198 = t138+t139+t190;
X0_6 = [q4rdot.*t180-q3rdot.*t184-q2rdot.*t186+q5rdot.*t195+q6rdot.*t194+q1rdot.*((t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2).^2+t129.^2);-q5rdot.*(-t169+t170+t158.*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)))-q1rdot.*t186-q4rdot.*t198+q3rdot.*t200-q6rdot.*t201+q2rdot.*((t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)).^2+t12.*t111+t13.*t111);-q1rdot.*t184-q4rdot.*t196+q2rdot.*t200-q6rdot.*t199+q3rdot.*(t12.*t108+t13.*t108+(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)).^2)-q5rdot.*(-t166+t167+t158.*(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)));q4rdot.*(t12.*t98+t13.*t98+(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2)).^2)+q1rdot.*t180-q3rdot.*t196-q2rdot.*t198+q6rdot.*t197-q5rdot.*(t163-t164+t158.*(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2)));-q2rdot.*(-t169+t170+t158.*(t130+t2.*(t32-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)))+q1rdot.*t195-q6rdot.*t204-q4rdot.*(t163-t164+t158.*(t116+t2.*(-t37-t44+t72+t75+t11.*(t59-t62).*8.23e-2)))-q3rdot.*(-t166+t167+t158.*(t126+t2.*(-t37-t44+t51-t53+t72+t75+t11.*(t59-t62).*8.23e-2)))+q5rdot.*(t148.^2+t149.^2+t158.^2);q1rdot.*t194+q4rdot.*t197-q3rdot.*t199-q2rdot.*t201-q5rdot.*t204+q6rdot.*(t134.^2+t135.^2+t140.^2)];
