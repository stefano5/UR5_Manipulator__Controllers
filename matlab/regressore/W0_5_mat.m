function W0_5 = W0_5_mat(q1,q2,q3,q4,q1dot,q2dot,q3dot,q4dot,q5dot,q1rdot,q2rdot,q3rdot,q4rdot,q5rdot)
%W0_5_MAT
%    W0_5 = W0_5_MAT(Q1,Q2,Q3,Q4,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q1RDOT,Q2RDOT,Q3RDOT,Q4RDOT,Q5RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:23

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
t12 = t3.*t4;
t13 = t3.*t8;
t14 = t4.*t7;
t15 = t7.*t8;
t25 = t3.*(1.7e+1./4.0e+1);
t26 = t7.*(1.7e+1./4.0e+1);
t34 = t2.*1.0915e-1;
t35 = t6.*1.0915e-1;
t16 = t6.*t15;
t17 = -t15;
t18 = t2.*t12;
t19 = t2.*t13;
t20 = t2.*t14;
t21 = t6.*t12;
t22 = t2.*t15;
t23 = t6.*t13;
t24 = t6.*t14;
t29 = t2.*t25;
t30 = t2.*t26;
t31 = t6.*t25;
t32 = t6.*t26;
t33 = t13+t14;
t37 = t12.*3.9225e-1;
t38 = t13.*3.9225e-1;
t39 = t14.*3.9225e-1;
t40 = t15.*3.9225e-1;
t41 = -t35;
t27 = -t21;
t28 = t2.*t17;
t36 = t12+t17;
t42 = t5.*t33;
t43 = t9.*t33;
t44 = -t40;
t45 = t19+t20;
t46 = t23+t24;
t47 = t16.*3.9225e-1;
t51 = t18.*3.9225e-1;
t52 = t19.*3.9225e-1;
t53 = t20.*3.9225e-1;
t54 = t21.*3.9225e-1;
t55 = t22.*3.9225e-1;
t56 = t23.*3.9225e-1;
t57 = t24.*3.9225e-1;
t48 = t5.*t36;
t49 = t9.*t36;
t50 = -t43;
t58 = -t47;
t59 = t18+t28;
t60 = t16+t27;
t61 = -t55;
t62 = t5.*t45;
t63 = t9.*t45;
t64 = t5.*t46;
t65 = t9.*t46;
t72 = t42.*9.465e-2;
t73 = t43.*9.465e-2;
t66 = t5.*t59;
t67 = t9.*t59;
t68 = -t63;
t69 = t5.*t60;
t70 = t9.*t60;
t74 = -t73;
t75 = t48.*9.465e-2;
t76 = t49.*9.465e-2;
t78 = t62.*9.465e-2;
t79 = t63.*9.465e-2;
t80 = t64.*9.465e-2;
t81 = t65.*9.465e-2;
t89 = t42+t49;
t90 = t48+t50;
t71 = -t70;
t77 = -t75;
t82 = -t79;
t83 = t66.*9.465e-2;
t84 = t67.*9.465e-2;
t85 = t69.*9.465e-2;
t86 = t70.*9.465e-2;
t91 = t62+t67;
t92 = t65+t69;
t93 = t66+t68;
t95 = t72+t76;
t96 = t74+t75;
t87 = -t83;
t88 = -t86;
t94 = t64+t71;
t97 = t78+t84;
t98 = t81+t85;
t99 = t82+t83;
t108 = t37+t44+t95;
t109 = t38+t39+t73+t77;
t113 = t91.*t95;
t116 = t91.*t96;
t117 = t92.*t96;
t123 = -t96.*(t63-t66);
t132 = t96.*(t63-t66);
t149 = t10.*t95.*t96;
t150 = t11.*t95.*t96;
t100 = t80+t88;
t101 = t6.*t98;
t102 = t2.*t99;
t104 = t41+t97;
t110 = t25+t108;
t111 = t26+t109;
t112 = t56+t57+t98;
t114 = t89.*t97;
t115 = t52+t53+t79+t87;
t119 = -t97.*(t43-t48);
t121 = t94.*t95;
t122 = -t98.*(t43-t48);
t124 = -t99.*(t43-t48);
t126 = t94.*t96;
t130 = t97.*(t43-t48);
t131 = t98.*(t43-t48);
t134 = t91.*t98;
t135 = t92.*t97;
t137 = t94.*t97;
t141 = t94.*t99;
t151 = t149.*2.0;
t152 = t150.*2.0;
t155 = -t149;
t156 = -t150;
t157 = t2.*t96.*t98;
t158 = t6.*t96.*t99;
t164 = t10.*t95.*t109;
t165 = t11.*t95.*t109;
t166 = t10.*t96.*t108;
t167 = t11.*t96.*t108;
t172 = t2.*t98.*t109;
t173 = t6.*t99.*t109;
t189 = t10.*t108.*t109;
t190 = t11.*t108.*t109;
t103 = -t101;
t105 = t34+t100;
t107 = t2.*t104;
t118 = t89.*t100;
t120 = -t114;
t125 = t6.*t112;
t127 = -t100.*(t43-t48);
t129 = t2.*t115;
t133 = t100.*(t43-t48);
t136 = t91.*t100;
t138 = -t135;
t139 = t32+t112;
t140 = -t100.*(t63-t66);
t142 = t30+t115;
t143 = -t137;
t144 = t100.*(t63-t66);
t147 = t51+t61+t104;
t168 = -t164;
t169 = -t165;
t170 = -t166;
t171 = -t167;
t174 = t11.*t95.*t111;
t175 = t10.*t95.*t111;
t176 = t10.*t96.*t110;
t177 = t11.*t96.*t110;
t178 = t2.*t96.*t112;
t179 = t6.*t96.*t115;
t183 = t2.*t98.*t111;
t184 = t6.*t99.*t111;
t191 = t189.*2.0;
t192 = t190.*2.0;
t197 = t10.*t109.*t110;
t198 = t10.*t108.*t111;
t199 = t11.*t109.*t110;
t200 = t11.*t108.*t111;
t201 = t2.*t109.*t112;
t202 = t6.*t109.*t115;
t204 = t2.*t111.*t112;
t205 = t6.*t111.*t115;
t210 = t116+t130;
t106 = t6.*t105;
t128 = -t118;
t145 = t6.*t139;
t146 = t2.*t142;
t148 = t54+t58+t105;
t153 = t2.*t147;
t159 = t29+t147;
t163 = t102+t103;
t180 = -t176;
t181 = -t177;
t182 = -t178;
t186 = t2.*t96.*t139;
t187 = t6.*t96.*t142;
t203 = -t201;
t206 = t2.*t109.*t139;
t207 = t6.*t109.*t142;
t208 = -t205;
t211 = t125+t129;
t212 = t126+t133;
t213 = t2.*t111.*t139;
t214 = t6.*t111.*t142;
t220 = t136+t143;
t227 = t6.*t95.*t210;
t230 = t99.*t210;
t233 = t6.*t108.*t210;
t236 = t6.*t110.*t210;
t237 = t115.*t210;
t244 = t142.*t210;
t256 = t113+t120+t124+t132;
t258 = t134+t138+t141+t144;
t154 = t6.*t148;
t160 = t31+t148;
t162 = t2.*t159;
t185 = t106+t107;
t188 = -t186;
t193 = t6.*t95.*t159;
t209 = -t207;
t215 = -t214;
t217 = t6.*t108.*t159;
t221 = t6.*t110.*t159;
t224 = t145+t146;
t228 = t2.*t95.*t212;
t231 = t98.*t212;
t232 = -t230;
t234 = t2.*t108.*t212;
t239 = t2.*t110.*t212;
t241 = t112.*t212;
t245 = t139.*t212;
t246 = -t220.*(t101-t102);
t247 = t220.*(t101-t102);
t255 = t117+t121+t128+t131;
t257 = t211.*t220;
t266 = t6.*t96.*t256;
t272 = t6.*t109.*t256;
t275 = t6.*t111.*t256;
t278 = t159.*t256;
t283 = t210.*t256.*2.0;
t286 = t220.*t258.*2.0;
t161 = t6.*t160;
t194 = t2.*t95.*t160;
t195 = -t193;
t216 = t2.*t108.*t160;
t219 = -t217;
t222 = t2.*t110.*t160;
t225 = t153+t154;
t229 = -t228;
t235 = -t234;
t238 = -t185.*(t101-t102);
t240 = t185.*(t101-t102).*-2.0;
t242 = t185.*(t101-t102).*2.0;
t243 = -t239;
t248 = t185.*t211;
t252 = t185.*t224;
t263 = t220.*t224;
t264 = t2.*t96.*t255;
t271 = t2.*t109.*t255;
t273 = -t272;
t274 = t2.*t111.*t255;
t276 = -t275;
t277 = t160.*t255;
t282 = t185.*t258;
t284 = t212.*t255.*2.0;
t196 = -t194;
t218 = -t216;
t223 = -t222;
t226 = t161+t162;
t249 = -t248;
t250 = -t225.*(t101-t102);
t251 = t225.*(t101-t102);
t253 = -t252;
t259 = t211.*t225;
t265 = t224.*t225;
t268 = -t264;
t279 = t151+t152+t242;
t285 = t157+t158+t194+t195;
t287 = t225.*t258;
t295 = t202+t203+t216+t219;
t309 = t231+t232+t277+t278;
t311 = t237+t241+t277+t278;
t313 = t244+t245+t277+t278;
t314 = t283+t284+t286;
t254 = -t226.*(t101-t102);
t260 = t259.*2.0;
t261 = -t259;
t267 = t211.*t226;
t269 = -t265;
t280 = q4rdot.*t279;
t288 = t172+t173+t193+t196;
t289 = t179+t182+t193+t196;
t290 = t183+t184+t193+t196;
t291 = t226.*t258;
t292 = t187+t188+t193+t196;
t296 = t204+t208+t217+t218;
t297 = t206+t209+t217+t218;
t298 = t213+t215+t221+t223;
t299 = t155+t156+t164+t165+t238+t250;
t301 = t166+t167+t168+t169+t248+t251;
t315 = q5rdot.*t314;
t317 = t227+t229+t247+t266+t268+t282;
t320 = t227+t229+t247+t271+t273+t287;
t322 = t233+t235+t257+t271+t273+t287;
t262 = -t260;
t270 = -t267;
t281 = -t280;
t300 = t155+t156+t174+t175+t238+t254;
t302 = q3rdot.*t301;
t303 = q4rdot.*t301;
t306 = t164+t165+t174+t175+t250+t254;
t307 = t170+t171+t174+t175+t249+t254;
t308 = t174+t175+t180+t181+t253+t254;
t316 = -t315;
t318 = q4rdot.*t317;
t319 = q5rdot.*t317;
t321 = t227+t229+t247+t274+t276+t291;
t323 = q3rdot.*t322;
t324 = q5rdot.*t322;
t325 = t233+t235+t257+t274+t276+t291;
t326 = t236+t243+t263+t274+t276+t291;
t293 = t191+t192+t262;
t304 = -t302;
t305 = -t303;
t310 = t189+t190+t198+t200+t261+t270;
t312 = t197+t198+t199+t200+t269+t270;
t294 = q3rdot.*t293;
W0_5 = [0.0;(q3dot.*(t294+t305+t324+q1rdot.*t297+q2rdot.*t312))./2.0+(q5dot.*(t316+t318+t323-q1rdot.*t313+q2rdot.*t326))./2.0-(q1dot.*(-q2rdot.*t298-q3rdot.*t297+q5rdot.*t313+q4rdot.*(t186-t187+t194+t195)+q1rdot.*(t139.*t160.*2.0+t142.*t159.*2.0)))./2.0+(q2dot.*(q1rdot.*t298+q3rdot.*t312+q5rdot.*t326-q4rdot.*(-t174-t175+t176+t177+t252+t226.*(t101-t102))+q2rdot.*(t224.*t226.*-2.0+t10.*t110.*t111.*2.0+t11.*t110.*t111.*2.0)))./2.0-(q4dot.*(t280+t302-t319+q1rdot.*(t186-t187+t194+t195)+q2rdot.*(-t174-t175+t176+t177+t252+t226.*(t101-t102))))./2.0;(q5dot.*(t316+t318+t323-q1rdot.*t311+q2rdot.*t325))./2.0+(q3dot.*(t294+t305+t324+q2rdot.*t310+q1rdot.*(t201-t202+t217+t218)))./2.0-(q1dot.*(-q2rdot.*t296+q5rdot.*t311+q4rdot.*(t178-t179+t194+t195)-q3rdot.*(t201-t202+t217+t218)+q1rdot.*(t112.*t160.*2.0+t115.*t159.*2.0)))./2.0+(q2dot.*(q1rdot.*t296+q3rdot.*t310+q5rdot.*t325+q2rdot.*(t198.*2.0+t200.*2.0-t267.*2.0)-q4rdot.*(t166+t167-t174-t175+t248+t226.*(t101-t102))))./2.0-(q4dot.*(t280+t302-t319+q1rdot.*(t178-t179+t194+t195)+q2rdot.*(t166+t167-t174-t175+t248+t226.*(t101-t102))))./2.0;q1dot.*(q4rdot.*t285-q3rdot.*t288-q2rdot.*t290+q5rdot.*t309+q1rdot.*(t98.*t160.*2.0-t99.*t159.*2.0)).*(-1.0./2.0)+(q2dot.*(q1rdot.*t290+q3rdot.*t306+q5rdot.*t321+q2rdot.*(t174.*2.0+t175.*2.0-t226.*(t101-t102).*2.0)-q4rdot.*(t149+t150-t174-t175+t185.*(t101-t102)+t226.*(t101-t102))))./2.0-(q4dot.*(t280-t319+q1rdot.*t285+q2rdot.*(t149+t150-t174-t175+t185.*(t101-t102)+t226.*(t101-t102))+q3rdot.*(t149+t150+t168+t169+t251+t185.*(t101-t102))))./2.0+(q3dot.*(q1rdot.*t288+q2rdot.*t306+q5rdot.*t320+q3rdot.*(t164.*2.0+t165.*2.0-t251.*2.0)-q4rdot.*(t149+t150+t168+t169+t251+t185.*(t101-t102))))./2.0+(q5dot.*(t316+t318-q1rdot.*t309+q2rdot.*t321+q3rdot.*t320))./2.0;0.0;0.0];
