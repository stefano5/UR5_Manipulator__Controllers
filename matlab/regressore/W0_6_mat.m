function W0_6 = W0_6_mat(q1,q2,q3,q4,q5,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q1rdot,q2rdot,q3rdot,q4rdot,q5rdot,q6rdot)
%W0_6_MAT
%    W0_6 = W0_6_MAT(Q1,Q2,Q3,Q4,Q5,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q1RDOT,Q2RDOT,Q3RDOT,Q4RDOT,Q5RDOT,Q6RDOT)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Feb-2022 19:10:36

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
t18 = t2.*t11;
t19 = t6.*t7;
t20 = t8.*t9;
t21 = t7.*t11;
t31 = t3.*(1.7e+1./4.0e+1);
t32 = t8.*(1.7e+1./4.0e+1);
t39 = t2.*t3.*(-1.7e+1./4.0e+1);
t41 = t2.*1.0915e-1;
t42 = t7.*1.0915e-1;
t22 = t7.*t20;
t23 = -t20;
t24 = t2.*t14;
t25 = t2.*t16;
t26 = t2.*t17;
t27 = t7.*t14;
t28 = t2.*t20;
t29 = t7.*t16;
t30 = t7.*t17;
t35 = t2.*t31;
t36 = t2.*t32;
t37 = t7.*t31;
t38 = t7.*t32;
t40 = t16+t17;
t44 = t14.*3.9225e-1;
t45 = t16.*3.9225e-1;
t46 = t17.*3.9225e-1;
t47 = t20.*3.9225e-1;
t50 = t15.*8.23e-2;
t51 = t18.*8.23e-2;
t52 = t19.*8.23e-2;
t54 = t21.*8.23e-2;
t33 = -t27;
t34 = t2.*t23;
t43 = t14+t23;
t48 = t5.*t40;
t49 = t10.*t40;
t53 = -t47;
t55 = t25+t26;
t56 = t29+t30;
t57 = t22.*3.9225e-1;
t61 = t24.*3.9225e-1;
t62 = t25.*3.9225e-1;
t63 = t26.*3.9225e-1;
t64 = t27.*3.9225e-1;
t65 = t28.*3.9225e-1;
t66 = t29.*3.9225e-1;
t67 = t30.*3.9225e-1;
t58 = t5.*t43;
t59 = t10.*t43;
t60 = -t49;
t68 = -t57;
t69 = t24+t34;
t70 = t22+t33;
t71 = -t61;
t72 = t5.*t55;
t73 = t10.*t55;
t74 = t5.*t56;
t75 = t10.*t56;
t82 = t48.*9.465e-2;
t83 = t49.*9.465e-2;
t76 = t5.*t69;
t77 = t10.*t69;
t78 = -t73;
t79 = t5.*t70;
t80 = t10.*t70;
t84 = -t83;
t85 = t58.*9.465e-2;
t86 = t59.*9.465e-2;
t88 = t72.*9.465e-2;
t89 = t73.*9.465e-2;
t90 = t74.*9.465e-2;
t91 = t75.*9.465e-2;
t101 = t48+t59;
t102 = t58+t60;
t114 = t11.*(t49-t58).*(-8.23e-2);
t116 = t11.*(t49-t58).*8.23e-2;
t81 = -t80;
t87 = -t85;
t92 = -t88;
t93 = -t89;
t94 = t76.*9.465e-2;
t95 = t77.*9.465e-2;
t96 = t79.*9.465e-2;
t97 = t80.*9.465e-2;
t103 = t72+t77;
t104 = t75+t79;
t105 = t76+t78;
t109 = t11.*t101.*8.23e-2;
t110 = -t6.*(t73-t76);
t111 = -t11.*(t73-t76);
t115 = t6.*(t73-t76);
t127 = t11.*(t73-t76).*(-8.23e-2);
t152 = t82+t86+t116;
t98 = -t94;
t99 = -t95;
t100 = -t97;
t106 = t74+t81;
t107 = t6.*t104;
t108 = t11.*t104;
t113 = -t109;
t118 = t19+t111;
t120 = t21+t115;
t121 = t11.*t103.*8.23e-2;
t126 = t115.*(-8.23e-2);
t129 = t115.*8.23e-2;
t133 = t52+t127;
t138 = t6.*t101.*t103.*8.23e-2;
t148 = t84+t85+t109;
t193 = t44+t53+t152;
t200 = t103.*t152;
t215 = -t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2);
t230 = -t101.*(-t52+t88+t95+t11.*(t73-t76).*8.23e-2);
t232 = (t49-t58).*(-t52+t88+t95+t11.*(t73-t76).*8.23e-2);
t237 = -t104.*(-t52+t88+t95+t11.*(t73-t76).*8.23e-2);
t246 = -t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t255 = -t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t281 = t19.*t101.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2).*(-8.23e-2);
t282 = t52.*t101.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t330 = -t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t112 = -t107;
t117 = t15+t108;
t122 = t107.*8.23e-2;
t123 = t108.*8.23e-2;
t124 = -t121;
t128 = t11.*t106.*8.23e-2;
t134 = t54+t129;
t139 = -t138;
t140 = t6.*t101.*t106.*8.23e-2;
t143 = t6.*t101.*t118.*8.23e-2;
t144 = t109.*t118;
t149 = t114.*t118;
t150 = t109.*t120;
t151 = t11.*t101.*t120.*(-8.23e-2);
t158 = t6.*t101.*t133;
t159 = t11.*t101.*t133;
t167 = -t11.*t133.*(t49-t58);
t170 = t11.*t133.*(t49-t58);
t174 = t93+t94+t121;
t177 = t11.*t106.*t133;
t191 = t45+t46+t83+t87+t113;
t194 = t92+t99+t133;
t196 = t31+t193;
t197 = t104.*t148;
t198 = t103.*t148;
t199 = t106.*t148;
t201 = -t148.*(t73-t76);
t202 = t106.*t152;
t204 = t148.*(t73-t76);
t207 = t6.*t12.*t101.*t148.*8.23e-2;
t208 = t6.*t13.*t101.*t148.*8.23e-2;
t239 = -t106.*(-t52+t88+t95+t11.*(t73-t76).*8.23e-2);
t258 = t13.*t148.*t152;
t260 = t12.*t148.*t152;
t271 = t12.*t148.*t193;
t273 = t13.*t148.*t193;
t351 = -t7.*t193.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t119 = t18+t112;
t125 = -t122;
t130 = -t128;
t131 = t50+t123;
t136 = t2.*t134;
t141 = t6.*t101.*t117.*8.23e-2;
t142 = t109.*t117;
t145 = t114.*t117;
t153 = t117.*t121;
t163 = t118.*t128;
t164 = t11.*t101.*t134;
t165 = -t134.*(t49-t58);
t166 = -t158;
t168 = -t159;
t175 = t106.*t134;
t178 = t2.*t174;
t179 = -t177;
t182 = t117.*t133;
t185 = t117.*t134;
t195 = t32+t191;
t205 = t42+t194;
t209 = -t207;
t210 = -t208;
t211 = t62+t63+t89+t98+t124;
t213 = -t174.*(t49-t58);
t221 = t106.*t174;
t236 = t7.*t134.*t148;
t241 = t6.*t12.*t101.*t191.*8.23e-2;
t242 = t6.*t13.*t101.*t191.*8.23e-2;
t259 = t7.*t134.*t191;
t261 = t258.*2.0;
t262 = t260.*2.0;
t266 = t149+t170;
t268 = t7.*t148.*t174;
t272 = t12.*t152.*t191;
t274 = t13.*t152.*t191;
t276 = -t271;
t278 = -t273;
t284 = t12.*t148.*t196;
t286 = t13.*t148.*t196;
t292 = t7.*t174.*t191;
t309 = t12.*t191.*t193;
t310 = t13.*t191.*t193;
t313 = t12.*t191.*t196;
t315 = t13.*t191.*t196;
t331 = -t7.*t148.*(t138+t134.*(t49-t58));
t333 = t7.*t148.*(t138+t134.*(t49-t58));
t341 = t198+t232;
t345 = -t7.*t148.*(t167+t116.*t118);
t346 = t7.*t148.*(t167+t116.*t118);
t353 = -t7.*t191.*(t138+t134.*(t49-t58));
t356 = -t7.*t196.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t361 = -t7.*t191.*(t167+t116.*t118);
t381 = (t138+t134.*(t49-t58)).*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t388 = (t167+t116.*t118).*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t132 = t51+t125;
t146 = t109.*t119;
t147 = t11.*t101.*t119.*(-8.23e-2);
t154 = t6.*t101.*t131;
t155 = t11.*t101.*t131;
t161 = -t11.*t131.*(t49-t58);
t169 = t11.*t131.*(t49-t58);
t172 = t11.*t103.*t131;
t176 = t91+t96+t130;
t181 = t118.*t131;
t186 = t120.*t131;
t187 = t119.*t133;
t188 = -t182;
t189 = -t185;
t192 = t90+t100+t131;
t217 = t2.*t211;
t223 = t36+t211;
t243 = t65+t71+t205;
t247 = t6.*t12.*t101.*t195.*8.23e-2;
t248 = t6.*t13.*t101.*t195.*8.23e-2;
t254 = t139+t165;
t264 = t144+t168;
t267 = t7.*t134.*t195;
t277 = -t272;
t279 = -t274;
t285 = t12.*t152.*t195;
t287 = t13.*t152.*t195;
t288 = -t284;
t290 = -t286;
t294 = -t292;
t297 = t7.*t148.*t211;
t302 = t7.*t174.*t195;
t311 = t309.*2.0;
t312 = t310.*2.0;
t314 = t12.*t193.*t195;
t316 = t13.*t193.*t195;
t319 = t7.*t191.*t211;
t322 = t7.*t195.*t211;
t357 = -t7.*t195.*(t138+t134.*(t49-t58));
t366 = -t7.*t195.*(t167+t116.*t118);
t369 = t52.*t101.*t341;
t379 = t134.*t341;
t389 = -t388;
t392 = t7.*t152.*t341;
t393 = t143+t151+t164+t166;
t397 = t174.*t341;
t400 = t7.*t193.*t341;
t404 = t7.*t196.*t341;
t408 = t211.*t341;
t421 = -t341.*(t167+t116.*t118);
t441 = t200+t204+t213+t230;
t135 = t7.*t132;
t156 = t11.*t101.*t132;
t157 = -t132.*(t49-t58);
t160 = -t154;
t162 = -t155;
t171 = t103.*t132;
t173 = -t172;
t180 = t7.*t176;
t184 = t118.*t132;
t190 = -t187;
t203 = t41+t192;
t214 = t66+t67+t176;
t216 = -t176.*(t49-t58);
t220 = t176.*(t49-t58);
t222 = t103.*t176;
t225 = t2.*t223;
t226 = t101.*t192;
t228 = -t192.*(t49-t58);
t231 = t192.*(t49-t58);
t233 = t103.*t192;
t234 = t2.*t132.*t148;
t235 = -t192.*(t73-t76);
t238 = t192.*(t73-t76);
t251 = t39+t243;
t257 = t2.*t132.*t191;
t263 = t145+t169;
t265 = t2.*t132.*t195;
t269 = t2.*t148.*t176;
t289 = -t285;
t291 = -t287;
t293 = t181+t188;
t295 = t2.*t176.*t191;
t300 = -t297;
t301 = t52.*t101.*t264;
t303 = t2.*t176.*t195;
t304 = -t302;
t306 = t7.*t148.*t223;
t318 = t134.*t264;
t324 = t7.*t191.*t223;
t334 = t7.*t195.*t223;
t343 = t7.*t152.*t264;
t349 = t174.*t264;
t360 = t7.*t193.*t264;
t364 = t7.*t196.*t264;
t370 = t211.*t264;
t375 = t223.*t264;
t387 = -t264.*(t138+t134.*(t49-t58));
t395 = -t392;
t396 = t264.*(t167+t116.*t118).*-2.0;
t410 = t223.*t341;
t418 = t7.*t148.*t393;
t430 = t7.*t191.*t393;
t433 = t7.*t195.*t393;
t443 = -t393.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t445 = t393.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t460 = t7.*t148.*t441;
t464 = t341.*t393;
t466 = t7.*t191.*t441;
t470 = t7.*t195.*t441;
t478 = -t441.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t479 = t441.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2);
t482 = t264.*t441;
t492 = t341.*t441.*2.0;
t137 = -t135;
t183 = -t180;
t206 = t7.*t203;
t219 = t7.*t214;
t224 = t38+t214;
t229 = -t226;
t240 = t64+t68+t203;
t253 = t140+t157;
t256 = t142+t162;
t270 = t171+t175;
t298 = -t295;
t299 = t2.*t148.*t214;
t305 = -t303;
t308 = -t306;
t320 = t2.*t191.*t214;
t323 = t2.*t195.*t214;
t340 = t2.*t148.*t263;
t342 = t199+t231;
t359 = t2.*t191.*t263;
t363 = t2.*t195.*t263;
t365 = t233+t239;
t386 = -t293.*(t135-t136);
t390 = t141+t147+t156+t160;
t402 = t153+t163+t173+t179;
t415 = t184+t186+t189+t190;
t432 = -t430;
t435 = -t433;
t453 = t221+t222+t237+t238;
t457 = t293.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2).*-2.0;
t462 = -t460;
t465 = -t464;
t468 = -t466;
t472 = -t470;
t483 = -t482;
t212 = -t206;
t218 = t136+t137;
t227 = t7.*t224;
t244 = t7.*t240;
t249 = t37+t240;
t283 = t178+t183;
t296 = t50.*t101.*t256;
t307 = t2.*t148.*t224;
t317 = t132.*t256;
t321 = -t320;
t326 = t2.*t191.*t224;
t327 = -t323;
t329 = t2.*t148.*t253;
t335 = t2.*t195.*t224;
t338 = t2.*t152.*t256;
t339 = t217+t219;
t344 = -t340;
t347 = t176.*t256;
t350 = t2.*t191.*t253;
t355 = t2.*t195.*t253;
t358 = t2.*t193.*t256;
t362 = t2.*t196.*t256;
t367 = t214.*t256;
t371 = t50.*t101.*t342;
t372 = t15.*t101.*t342.*(-8.23e-2);
t373 = t224.*t256;
t377 = t132.*t342;
t384 = t253.*t256;
t391 = t256.*t263.*2.0;
t394 = t2.*t152.*t342;
t398 = t176.*t342;
t401 = t2.*t193.*t342;
t405 = t2.*t196.*t342;
t406 = (t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)).*(t135-t136);
t409 = t214.*t342;
t411 = t224.*t342;
t412 = t270.*t293;
t416 = -t365.*(t135-t136);
t417 = t2.*t148.*t390;
t419 = t263.*t342;
t423 = -t270.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t424 = t2.*t191.*t390;
t429 = t2.*t195.*t390;
t442 = t197+t202+t220+t229;
t463 = t342.*t390;
t473 = (t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)).*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2);
t476 = -t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2);
t480 = -t415.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t487 = t365.*t415;
t494 = t293.*t453;
t498 = -t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t499 = t365.*t453.*2.0;
t245 = -t244;
t250 = t7.*t249;
t275 = t50.*t101.*t249;
t280 = t15.*t101.*t249.*(-8.23e-2);
t325 = t2.*t152.*t249;
t328 = -t326;
t332 = -t329;
t336 = t212+t215;
t337 = -t335;
t348 = t2.*t193.*t249;
t352 = t225+t227;
t354 = t2.*t196.*t249;
t368 = -t367;
t374 = -t373;
t378 = t249.*t253;
t380 = -t377;
t383 = t249.*t263;
t399 = -t398;
t403 = -t401;
t407 = -t405;
t413 = t283.*t293;
t420 = -t419;
t422 = (t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*(t135-t136);
t425 = -t424;
t426 = -t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t427 = t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)).*-2.0;
t431 = -t429;
t434 = t293.*t339;
t437 = t283.*t365;
t438 = t249.*t390;
t440 = -t270.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t444 = -t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t446 = -t283.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t447 = t283.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t452 = t339.*t365;
t454 = -t339.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t455 = t339.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*-2.0;
t461 = t2.*t148.*t442;
t467 = t2.*t191.*t442;
t471 = t2.*t195.*t442;
t475 = t249.*t442;
t481 = t256.*t442;
t484 = (t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2);
t488 = -t415.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t493 = t342.*t442.*2.0;
t495 = -t494;
t501 = -t453.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t502 = t453.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t517 = t391+t396+t457;
t518 = -q6rdot.*(-t391+t293.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2).*2.0+t264.*(t167+t116.*t118).*2.0);
t519 = q6rdot.*(-t391+t293.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2).*2.0+t264.*(t167+t116.*t118).*2.0);
t552 = t296+t301+t386+t417+t418+t480;
t559 = t384+t387+t412+t463+t465+t487;
t252 = -t250;
t376 = t245+t246;
t385 = -t383;
t414 = -t413;
t428 = (t135-t136).*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t436 = t293.*t352;
t439 = -t438;
t448 = -t270.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t449 = -t352.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2));
t450 = -t283.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t451 = t283.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t456 = t352.*t365;
t458 = -t352.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t459 = -t339.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t469 = -t467;
t474 = -t471;
t477 = -t475;
t485 = t234+t236+t280+t282;
t486 = (t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2);
t489 = t257+t259+t275+t281;
t490 = -t415.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t491 = t265+t267+t275+t281;
t496 = t261+t262+t427;
t500 = t268+t269+t325+t330;
t503 = -t453.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t504 = t453.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2));
t505 = t294+t298+t325+t330;
t506 = t299+t300+t325+t330;
t507 = t304+t305+t325+t330;
t508 = t307+t308+t325+t330;
t509 = t311+t312+t455;
t511 = t319+t321+t348+t351;
t512 = t322+t327+t348+t351;
t513 = t324+t328+t348+t351;
t514 = t334+t337+t354+t356;
t516 = t368+t370+t383+t388;
t520 = t374+t375+t383+t388;
t521 = t378+t379+t380+t381;
t522 = t209+t210+t241+t242+t406+t422;
t526 = t258+t260+t277+t279+t426+t446;
t528 = t272+t274+t276+t278+t444+t447;
t529 = -q3rdot.*(t271+t273+t277+t279+t446+t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)));
t530 = -q4rdot.*(t271+t273+t277+t279+t446+t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)));
t531 = q3rdot.*(t271+t273+t277+t279+t446+t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)));
t535 = t332+t333+t369+t372+t416+t423;
t538 = t350+t353+t369+t372+t416+t440;
t544 = t408+t409+t475+t479;
t545 = t410+t411+t475+t479;
t548 = t358+t359+t360+t361+t434+t484;
t555 = t492+t493+t499;
t557 = t296+t301+t386+t425+t432+t488;
t560 = t394+t395+t437+t461+t462+t498;
t561 = -q4rdot.*(t392-t394-t437+t460-t461+t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)));
t562 = -q5rdot.*(t392-t394-t437+t460-t461+t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)));
t565 = t400+t403+t452+t467+t468+t502;
t571 = t420+t421+t476+t481+t483+t495;
t572 = -q5rdot.*(t419-t481+t482+t494+t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2)+t341.*(t167+t116.*t118));
t573 = -q6rdot.*(t419-t481+t482+t494+t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2)+t341.*(t167+t116.*t118));
t382 = t252+t255;
t497 = q4rdot.*t496;
t510 = q3rdot.*t509;
t515 = t347+t349+t385+t389;
t523 = t209+t210+t247+t248+t406+t428;
t524 = t317+t318+t439+t445;
t525 = t241+t242+t247+t248+t422+t428;
t527 = t258+t260+t289+t291+t426+t450;
t532 = t276+t278+t285+t287+t444+t451;
t533 = t272+t274+t285+t287+t447+t451;
t534 = t285+t287+t288+t290+t449+t451;
t536 = t309+t310+t314+t316+t454+t459;
t537 = t397+t399+t477+t478;
t539 = t338+t343+t344+t346+t414+t473;
t540 = t313+t314+t315+t316+t458+t459;
t543 = t355+t357+t369+t372+t416+t448;
t546 = t338+t343+t359+t361+t414+t484;
t547 = t338+t343+t363+t366+t414+t486;
t549 = q3rdot.*t548;
t550 = q6rdot.*t548;
t553 = t358+t360+t363+t366+t434+t486;
t554 = t362+t363+t364+t366+t436+t486;
t556 = q5rdot.*t555;
t558 = t296+t301+t386+t431+t435+t490;
t563 = t394+t395+t437+t466+t469+t501;
t564 = t394+t395+t437+t470+t474+t503;
t566 = q3rdot.*t565;
t567 = q5rdot.*t565;
t569 = t400+t403+t452+t471+t472+t504;
t570 = t404+t407+t456+t471+t472+t504;
t541 = q4rdot.*t539;
t542 = q6rdot.*t539;
t551 = -t550;
t568 = -t566;
W0_6 = [0.0;q4dot.*(t497+t531+t542+t562+q2rdot.*(t284+t286+t289+t291+t450+t352.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))-q1rdot.*(t306-t307-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))).*(-1.0./2.0)+(q5dot.*(-t556+t566-q1rdot.*t545+q2rdot.*t570+q4rdot.*(t392-t394-t437+t460-t461+t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q6rdot.*(t419-t481+t482+t494+t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2)+t341.*(t167+t116.*t118))))./2.0-(q1dot.*(q2rdot.*t514+q3rdot.*t513-q6rdot.*t520+q5rdot.*t545+q1rdot.*(t224.*t249.*2.0+t223.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2).*2.0)-q4rdot.*(t306-t307-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))))./2.0+(q3dot.*(t510+t530+t551+t567-q1rdot.*t513+q2rdot.*t540))./2.0-(q6dot.*(t519+t541+t549+t572-q1rdot.*t520+q2rdot.*t554))./2.0-(q2dot.*(q1rdot.*t514-q3rdot.*t540+q6rdot.*t554-q5rdot.*t570-q2rdot.*(t352.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*-2.0+t12.*t195.*t196.*2.0+t13.*t195.*t196.*2.0)+q4rdot.*(t284+t286+t289+t291+t450+t352.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))))./2.0;q4dot.*(t497+t531+t542+t562+q2rdot.*(t271+t273+t289+t291+t450+t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))-q1rdot.*(t297-t299-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))).*(-1.0./2.0)-(q2dot.*(q1rdot.*t512-q3rdot.*t536+q6rdot.*t553-q5rdot.*t569+q4rdot.*(t271+t273+t289+t291+t450+t339.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))-q2rdot.*(t314.*2.0+t316.*2.0-t339.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2)).*2.0)))./2.0+(q5dot.*(-t556+t566-q1rdot.*t544+q2rdot.*t569+q4rdot.*(t392-t394-t437+t460-t461+t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q6rdot.*(t419-t481+t482+t494+t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2)+t341.*(t167+t116.*t118))))./2.0-(q1dot.*(q2rdot.*t512+q3rdot.*t511-q6rdot.*t516+q5rdot.*t544+q1rdot.*(t214.*t249.*2.0+t211.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2).*2.0)-q4rdot.*(t297-t299-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))))./2.0+(q3dot.*(t510+t530+t551+t567-q1rdot.*t511+q2rdot.*t536))./2.0-(q6dot.*(t519+t541+t549+t572-q1rdot.*t516+q2rdot.*t553))./2.0;(q5dot.*(-t556-q1rdot.*(-t397+t398+t475+t479)+q4rdot.*(t392-t394-t437+t460-t461+t453.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q3rdot.*(t392-t394-t437+t467+t468+t502)+q2rdot.*(t392-t394-t437+t471+t472+t504)+q6rdot.*(t419-t481+t482+t494+t365.*(t172+t177-t11.*t103.*t117.*8.23e-2-t11.*t106.*t118.*8.23e-2)+t341.*(t167+t116.*t118))))./2.0-(q1dot.*(q4rdot.*t500+q6rdot.*t515-q3rdot.*(t292+t295-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))-q2rdot.*(t302+t303-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+q5rdot.*(-t397+t398+t475+t479)+q1rdot.*(t176.*t249.*2.0-t174.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2).*2.0)))./2.0-(q6dot.*(t519+t541+t572+q1rdot.*t515+q2rdot.*t547+q3rdot.*t546))./2.0+(q3dot.*(q4rdot.*(-t258-t260+t272+t274+t447+t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q2rdot.*t533-q6rdot.*t546+q1rdot.*(t292+t295-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+q5rdot.*(t392-t394-t437+t467+t468+t502)+q3rdot.*(t272.*2.0+t274.*2.0+t447.*2.0)))./2.0+(q2dot.*(q4rdot.*(-t258-t260+t285+t287+t451+t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q3rdot.*t533-q6rdot.*t547+q1rdot.*(t302+t303-t325+t7.*t152.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+q5rdot.*(t392-t394-t437+t471+t472+t504)+q2rdot.*(t285.*2.0+t287.*2.0+t451.*2.0)))./2.0-(q4dot.*(t497+t542+t562-q3rdot.*(-t258-t260+t272+t274+t447+t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))-q2rdot.*(-t258-t260+t285+t287+t451+t283.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2)))+q1rdot.*t500))./2.0;(q6dot.*(q6rdot.*(t256.*t390.*2.0+t264.*t393.*2.0-t293.*t415.*2.0)-q1rdot.*t524+q4rdot.*t552+q5rdot.*t559-q3rdot.*(-t301+t424+t430+t415.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t293.*(t135-t136)-t15.*t101.*t256.*8.23e-2)-q2rdot.*(-t301+t429+t433+t293.*(t135-t136)+t415.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))-t15.*t101.*t256.*8.23e-2)))./2.0-(q3dot.*(-q1rdot.*t489+q4rdot.*t522+q2rdot.*t525+q6rdot.*(-t301+t424+t430+t415.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t293.*(t135-t136)-t15.*t101.*t256.*8.23e-2)+q3rdot.*(t422.*2.0+t6.*t12.*t101.*t191.*1.646e-1+t6.*t13.*t101.*t191.*1.646e-1)-q5rdot.*(-t350+t371+t270.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t365.*(t135-t136)+t7.*t191.*(t138+t134.*(t49-t58))-t19.*t101.*t341.*8.23e-2)))./2.0-(q4dot.*(q1rdot.*t485+q2rdot.*t523+q3rdot.*t522-q6rdot.*t552-q4rdot.*(t406.*-2.0+t6.*t12.*t101.*t148.*1.646e-1+t6.*t13.*t101.*t148.*1.646e-1)-q5rdot.*(t329+t331+t365.*(t135-t136)+t270.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2))+t15.*t101.*t342.*8.23e-2-t19.*t101.*t341.*8.23e-2)))./2.0+(q5dot.*(q1rdot.*t521+q6rdot.*t559+q3rdot.*(-t350+t371+t270.*(t244+t2.*(-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t365.*(t135-t136)+t7.*t191.*(t138+t134.*(t49-t58))-t19.*t101.*t341.*8.23e-2)+q5rdot.*(t341.*(t138+t134.*(t49-t58)).*2.0+t253.*t342.*2.0-t270.*t365.*2.0)+q2rdot.*(-t355+t371+t365.*(t135-t136)+t270.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t7.*t195.*(t138+t134.*(t49-t58))-t19.*t101.*t341.*8.23e-2)+q4rdot.*(t329+t331+t365.*(t135-t136)+t270.*(t206+t2.*(-t42-t52+t88+t95+t11.*(t73-t76).*8.23e-2))+t15.*t101.*t342.*8.23e-2-t19.*t101.*t341.*8.23e-2)))./2.0-(q1dot.*(q4rdot.*t485-q3rdot.*t489-q2rdot.*t491-q5rdot.*t521+q6rdot.*t524+q1rdot.*(t132.*t249.*2.0-t134.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2).*2.0)))./2.0-(q2dot.*(-q1rdot.*t491+q4rdot.*t523+q3rdot.*t525+q2rdot.*(t428.*2.0+t6.*t12.*t101.*t195.*1.646e-1+t6.*t13.*t101.*t195.*1.646e-1)+q6rdot.*(-t301+t429+t433+t293.*(t135-t136)+t415.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))-t15.*t101.*t256.*8.23e-2)-q5rdot.*(-t355+t371+t365.*(t135-t136)+t270.*(t250+t2.*(t35-t42-t52+t61-t65+t88+t95+t11.*(t73-t76).*8.23e-2))+t7.*t195.*(t138+t134.*(t49-t58))-t19.*t101.*t341.*8.23e-2)))./2.0;0.0];
