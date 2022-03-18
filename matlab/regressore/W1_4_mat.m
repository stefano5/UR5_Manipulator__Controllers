function W1_4 = W1_4_mat(q1,q2,q3,q4,q1dot,q2dot,q3dot,q4dot,q1rdot,q2rdot,q3rdot,q4rdot)
%W1_4_MAT
%    W1_4 = W1_4_MAT(Q1,Q2,Q3,Q4,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q1RDOT,Q2RDOT,Q3RDOT,Q4RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:41

t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = sin(q1);
t7 = sin(q2);
t8 = sin(q3);
t9 = sin(q4);
t10 = t2.^2;
t11 = t6.^2;
t12 = q2dot.*t2;
t13 = q3dot.*t2;
t14 = q4dot.*t2;
t15 = q2dot.*t6;
t16 = q3dot.*t6;
t17 = q4dot.*t6;
t18 = t3.*t4;
t19 = t3.*t8;
t20 = t4.*t7;
t21 = t7.*t8;
t31 = t3.*(1.7e+1./4.0e+1);
t32 = t7.*(1.7e+1./4.0e+1);
t39 = t2.*t3.*(-1.7e+1./4.0e+1);
t41 = t2.*1.0915e-1;
t42 = t6.*1.0915e-1;
t22 = t6.*t21;
t23 = -t21;
t24 = t2.*t18;
t25 = t2.*t19;
t26 = t2.*t20;
t27 = t6.*t18;
t28 = t2.*t21;
t29 = t6.*t19;
t30 = t6.*t20;
t35 = t2.*t31;
t36 = t2.*t32;
t37 = t6.*t31;
t38 = t6.*t32;
t40 = t19+t20;
t43 = t12+t13+t14;
t44 = t15+t16+t17;
t46 = t18.*3.9225e-1;
t47 = t19.*3.9225e-1;
t48 = t20.*3.9225e-1;
t49 = t21.*3.9225e-1;
t33 = -t27;
t34 = t2.*t23;
t45 = t18+t23;
t50 = t5.*t40;
t51 = t9.*t40;
t52 = -t49;
t53 = t25+t26;
t54 = t29+t30;
t55 = t22.*3.9225e-1;
t56 = t6.*t43;
t57 = t2.*t44;
t61 = t24.*3.9225e-1;
t62 = t25.*3.9225e-1;
t63 = t26.*3.9225e-1;
t64 = t27.*3.9225e-1;
t65 = t28.*3.9225e-1;
t66 = t29.*3.9225e-1;
t67 = t30.*3.9225e-1;
t83 = t47+t48;
t58 = t5.*t45;
t59 = t9.*t45;
t60 = -t51;
t68 = -t55;
t69 = t24+t34;
t70 = t22+t33;
t71 = -t61;
t72 = t5.*t53;
t73 = t9.*t53;
t74 = t5.*t54;
t75 = t9.*t54;
t76 = -t57;
t84 = t46+t52;
t86 = t13.*t83;
t87 = t62+t63;
t88 = t16.*t83;
t89 = t66+t67;
t103 = t32+t83;
t77 = t5.*t69;
t78 = t9.*t69;
t79 = -t73;
t80 = t5.*t70;
t81 = t9.*t70;
t85 = t50+t59;
t90 = t58+t60;
t91 = t12.*t84;
t92 = t13.*t84;
t93 = t15.*t84;
t94 = t16.*t84;
t95 = q1dot.*t87;
t96 = q1dot.*t89;
t97 = t56+t76;
t98 = t2.*t87;
t99 = q1dot.*t10.*t84;
t100 = t6.*t89;
t101 = q1dot.*t11.*t84;
t104 = t31+t84;
t110 = t12.*t103;
t111 = t15.*t103;
t116 = t36+t87;
t118 = t38+t89;
t126 = t42+t65+t71;
t127 = t41+t64+t68;
t128 = -t43.*(t51-t58);
t129 = -t44.*(t51-t58);
t130 = t44.*(t51-t58);
t82 = -t81;
t102 = -t95;
t105 = t72+t78;
t106 = t75+t80;
t108 = t77+t79;
t113 = t12.*t104;
t114 = t15.*t104;
t115 = -q1dot.*(t73-t77);
t119 = q1dot.*t116;
t120 = q1dot.*t118;
t121 = t43.*t85;
t122 = t44.*t85;
t123 = t2.*t116;
t124 = t6.*t118;
t131 = t2.*t126;
t132 = t6.*t127;
t137 = -t43.*(t73-t77);
t138 = t37+t127;
t139 = t43.*(t73-t77);
t140 = t39+t126;
t148 = t98+t100;
t158 = t91+t92+t96;
t107 = q1dot.*t105;
t109 = q1dot.*t106;
t112 = t74+t82;
t125 = -t119;
t133 = -t132;
t134 = t44.*t106;
t135 = t43.*t105;
t141 = q1dot.*t138;
t142 = q1dot.*t140;
t143 = t6.*t138;
t145 = t2.*t140;
t149 = q2dot.*t148;
t150 = q3dot.*t148;
t152 = t115+t122;
t155 = t123+t124;
t160 = t93+t94+t102;
t161 = t97.*t148;
t169 = t92+t113+t120;
t184 = t85.*t158;
t188 = -t158.*(t51-t58);
t117 = q1dot.*t112;
t136 = t44.*t112;
t144 = -t141;
t146 = -t142;
t147 = -t143;
t151 = t109+t121;
t153 = t107+t130;
t156 = q2dot.*t155;
t157 = t131+t133;
t163 = -t161;
t164 = t134+t139;
t167 = t6.*t83.*t152;
t168 = t6.*t84.*t152;
t175 = t94+t114+t125;
t178 = -q2dot.*(t143-t145);
t180 = t6.*t103.*t152;
t187 = t85.*t160;
t189 = -t160.*(t51-t58);
t191 = t140.*t152;
t195 = t149+t150;
t196 = t85.*t169;
t198 = -t169.*(t51-t58);
t154 = t117+t128;
t159 = q3dot.*t157;
t162 = t135+t136;
t165 = t2.*t83.*t151;
t166 = t2.*t84.*t151;
t171 = t6.*t83.*t153;
t172 = t145+t147;
t174 = t6.*t84.*t153;
t179 = t2.*t103.*t151;
t182 = t6.*t103.*t153;
t185 = t86+t110+t144;
t186 = t88+t111+t146;
t190 = t138.*t151;
t193 = t140.*t153;
t197 = t85.*t175;
t199 = -t175.*(t51-t58);
t204 = t150+t156;
t213 = t99+t101+t163;
t214 = t148.*t164;
t217 = t105.*t195;
t218 = t106.*t195;
t219 = -t195.*(t73-t77);
t220 = t112.*t195;
t221 = t195.*(t73-t77);
t228 = t157.*t164;
t231 = -t164.*(t143-t145);
t170 = t2.*t83.*t154;
t173 = t2.*t84.*t154;
t176 = -t171;
t181 = t2.*t103.*t154;
t183 = -t182;
t192 = t138.*t154;
t200 = t85.*t185;
t201 = t85.*t186;
t202 = -t185.*(t51-t58);
t205 = -t186.*(t51-t58);
t207 = t106.*t185;
t208 = t105.*t186;
t209 = t112.*t185;
t210 = -t186.*(t73-t77);
t212 = t148.*t162;
t216 = (q3rdot.*t213)./2.0;
t222 = t105.*t204;
t223 = t106.*t204;
t224 = -t204.*(t73-t77);
t225 = t112.*t204;
t226 = t157.*t162;
t227 = t204.*(t73-t77);
t229 = t159+t178;
t230 = -t162.*(t143-t145);
t177 = -t173;
t194 = -t192;
t203 = -t200;
t206 = -t201;
t211 = -t208;
t215 = -t212;
t232 = t105.*t229;
t233 = t106.*t229;
t234 = -t229.*(t73-t77);
t235 = t112.*t229;
t243 = -t2.*(t200+t229.*(t73-t77));
t245 = t2.*(t200+t229.*(t73-t77));
t250 = -t2.*(t200-t217+t158.*(t51-t58)+t229.*(t73-t77));
t258 = -t2.*(t200-t222+t169.*(t51-t58)+t229.*(t73-t77));
t262 = t166+t168+t170+t176+t214+t226;
t236 = -t233;
t237 = t201+t233;
t238 = t202+t232;
t240 = t203+t234;
t241 = t205+t235;
t263 = t165+t167+t174+t177+t215+t228;
t264 = (q3rdot.*t262)./2.0;
t239 = t6.*t237;
t242 = t2.*t238;
t244 = t6.*t241;
t246 = t184+t221+t238;
t247 = t188+t217+t240;
t248 = t187+t218+t241;
t252 = t189+t206+t220+t236;
t253 = -t6.*(-t220+t237+t160.*(t51-t58));
t254 = t196+t227+t238;
t255 = t198+t222+t240;
t256 = t197+t223+t241;
t260 = t199+t206+t225+t236;
t261 = -t6.*(-t225+t237+t175.*(t51-t58));
t265 = -t264;
t266 = (q3rdot.*t263)./2.0;
t249 = t2.*t246;
t251 = t6.*t248;
t257 = t2.*t254;
t259 = t6.*t256;
t267 = t239+t245;
t268 = t242+t244;
t269 = -t266;
t271 = t250+t253;
t273 = t258+t261;
t270 = t249+t251;
t272 = t257+t259;
W1_4 = reshape([0.0,t265+(q1rdot.*(t192-t193+t116.*t152-t118.*t151))./2.0+(q1rdot.*(t208-t209-t106.*t169+t175.*(t73-t77)))./2.0-(q2rdot.*t272)./2.0-(q3rdot.*t272)./2.0-(q4rdot.*t272)./2.0-(q2rdot.*(t181+t183+t230+t155.*t164+t2.*t104.*t151+t6.*t104.*t152))./2.0,t265+(q1rdot.*(t192-t193+t87.*t152-t89.*t151))./2.0+(q1rdot.*(t208-t209-t106.*t158+t160.*(t73-t77)))./2.0-(q2rdot.*t270)./2.0-(q3rdot.*t270)./2.0-(q4rdot.*t270)./2.0-(q2rdot.*(t166+t168+t181+t183+t214+t230))./2.0,q2rdot.*t268.*(-1.0./2.0)-(q3rdot.*t268)./2.0-(q4rdot.*t268)./2.0+(q1rdot.*(t192-t193))./2.0+(q1rdot.*(t208-t209))./2.0-(q3rdot.*(t170+t176+t226))./2.0+(q2rdot.*(-t181+t182+t162.*(t143-t145)))./2.0,0.0,0.0,0.0,t216+(q1rdot.*(t2.*t120+t6.*t125))./2.0+(q1rdot.*(t2.*t169+t6.*t175))./2.0+(q2rdot.*(-t97.*t155+q1dot.*t10.*t104+q1dot.*t11.*t104))./2.0,t216+(q2rdot.*t213)./2.0+(q1rdot.*(t2.*t96+t6.*t102))./2.0+(q1rdot.*(t2.*t158+t6.*t160))./2.0,0.0,0.0,0.0,0.0,t269-(q2rdot.*(t179+t180+t231-t155.*t162-t2.*t104.*t154+t6.*t104.*t153))./2.0-(q2rdot.*(t2.*(t200-t222+t169.*(t51-t58)+t229.*(t73-t77))+t6.*(-t225+t237+t175.*(t51-t58))))./2.0-(q3rdot.*(t2.*(t200-t222+t169.*(t51-t58)+t229.*(t73-t77))+t6.*(-t225+t237+t175.*(t51-t58))))./2.0-(q4rdot.*(t2.*(t200-t222+t169.*(t51-t58)+t229.*(t73-t77))+t6.*(-t225+t237+t175.*(t51-t58))))./2.0+(q1rdot.*(t190+t191+t116.*t153+t118.*t154))./2.0-(q1rdot.*(t207+t210+t105.*t175-t112.*t169))./2.0,t269+(q2rdot.*(t173-t174-t179-t180+t212+t164.*(t143-t145)))./2.0-(q2rdot.*(t2.*(t200-t217+t158.*(t51-t58)+t229.*(t73-t77))+t6.*(-t220+t237+t160.*(t51-t58))))./2.0-(q3rdot.*(t2.*(t200-t217+t158.*(t51-t58)+t229.*(t73-t77))+t6.*(-t220+t237+t160.*(t51-t58))))./2.0-(q4rdot.*(t2.*(t200-t217+t158.*(t51-t58)+t229.*(t73-t77))+t6.*(-t220+t237+t160.*(t51-t58))))./2.0+(q1rdot.*(t190+t191+t87.*t153+t89.*t154))./2.0-(q1rdot.*(t207+t210+t105.*t160-t112.*t158))./2.0,(q1rdot.*(t190+t191))./2.0-(q1rdot.*(t207+t210))./2.0-(q2rdot.*t267)./2.0-(q3rdot.*t267)./2.0-(q4rdot.*t267)./2.0-(q3rdot.*(t165+t167+t228))./2.0-(q2rdot.*(t179+t180+t231))./2.0,0.0,0.0],[6,3]);