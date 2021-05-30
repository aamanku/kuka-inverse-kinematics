function out1 = jointjacibian_7(in1,in2)
%JOINTJACIBIAN_7
%    OUT1 = JOINTJACIBIAN_7(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    12-May-2020 10:56:26

obx = in2(1,:);
oby = in2(2,:);
obz = in2(3,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
t2 = cos(q1);
t3 = sin(q2);
t4 = cos(q4);
t5 = sin(q1);
t6 = sin(q3);
t7 = t5.*t6;
t8 = cos(q2);
t9 = cos(q3);
t12 = t2.*t8.*t9;
t10 = t7-t12;
t11 = sin(q4);
t13 = t2.*t3.*(2.0./5.0);
t14 = cos(q6);
t15 = t10.*t11;
t16 = t2.*t3.*t4;
t17 = t15+t16;
t18 = t14.*t17.*(6.3e1./5.0e2);
t19 = sin(q6);
t20 = cos(q5);
t21 = t4.*t10;
t54 = t2.*t3.*t11;
t22 = t21-t54;
t23 = t20.*t22;
t24 = sin(q5);
t25 = t5.*t9;
t26 = t2.*t6.*t8;
t27 = t25+t26;
t28 = t24.*t27;
t29 = t23+t28;
t30 = t10.*t11.*(2.0./5.0);
t31 = t2.*t3.*t4.*(2.0./5.0);
t55 = t19.*t29.*(6.3e1./5.0e2);
t32 = -oby+t13+t18+t30+t31-t55;
t33 = t2.*t6;
t34 = t5.*t8.*t9;
t35 = t33+t34;
t36 = t3.*t5.*(2.0./5.0);
t37 = t11.*t35;
t49 = t3.*t4.*t5;
t38 = t37-t49;
t39 = t4.*t35;
t40 = t3.*t5.*t11;
t41 = t39+t40;
t42 = t20.*t41;
t43 = t2.*t9;
t51 = t5.*t6.*t8;
t44 = t43-t51;
t45 = t24.*t44;
t46 = t42+t45;
t47 = t19.*t46.*(6.3e1./5.0e2);
t48 = t3.*t4.*t5.*(2.0./5.0);
t50 = t14.*t38.*(6.3e1./5.0e2);
t52 = t11.*t35.*(2.0./5.0);
t53 = obz-t36-t47-t48+t50+t52;
t59 = t8.*(2.0./5.0);
t60 = t4.*t8.*(2.0./5.0);
t61 = t8.*t11;
t62 = t3.*t4.*t9;
t63 = t61-t62;
t64 = t20.*t63;
t65 = t3.*t6.*t24;
t66 = t64+t65;
t67 = t19.*t66.*(6.3e1./5.0e2);
t68 = t4.*t8;
t69 = t3.*t9.*t11;
t70 = t68+t69;
t71 = t14.*t70.*(6.3e1./5.0e2);
t72 = t3.*t9.*t11.*(2.0./5.0);
t73 = -obx+t59+t60+t67+t71+t72+1.7e1./5.0e1;
t56 = abs(t73);
t57 = abs(t53);
t58 = abs(t32);
t74 = sign(t32);
t75 = sign(t53);
t76 = t56.^2;
t77 = t57.^2;
t78 = t58.^2;
t79 = t76+t77+t78;
t80 = 1.0./sqrt(t79);
t81 = sign(t73);
out1 = [t80.*(t57.*t75.*(t13+t18+t30+t31-t55).*2.0+t58.*t74.*(t36+t47+t48-t50-t52).*2.0).*(-1.0./2.0),t80.*(t58.*t74.*(t14.*(t2.*t4.*t8+t2.*t3.*t9.*t11).*(6.3e1./5.0e2)+t2.*t8.*(2.0./5.0)+t19.*(t20.*(t2.*t8.*t11-t2.*t3.*t4.*t9)+t2.*t3.*t6.*t24).*(6.3e1./5.0e2)+t2.*t4.*t8.*(2.0./5.0)+t2.*t3.*t9.*t11.*(2.0./5.0)).*-2.0+t57.*t75.*(t14.*(t4.*t5.*t8+t3.*t5.*t9.*t11).*(6.3e1./5.0e2)+t5.*t8.*(2.0./5.0)+t19.*(t20.*(t5.*t8.*t11-t3.*t4.*t5.*t9)+t3.*t5.*t6.*t24).*(6.3e1./5.0e2)+t4.*t5.*t8.*(2.0./5.0)+t3.*t5.*t9.*t11.*(2.0./5.0)).*2.0+t56.*t81.*(t3.*(2.0./5.0)+t19.*(t20.*(t3.*t11+t4.*t8.*t9)-t6.*t8.*t24).*(6.3e1./5.0e2)+t14.*(t3.*t4-t8.*t9.*t11).*(6.3e1./5.0e2)+t3.*t4.*(2.0./5.0)-t8.*t9.*t11.*(2.0./5.0)).*2.0).*(-1.0./2.0),t80.*(t56.*t81.*(t19.*(t3.*t9.*t24+t3.*t4.*t6.*t20).*(-6.3e1./5.0e2)+t3.*t6.*t11.*(2.0./5.0)+t3.*t6.*t11.*t14.*(6.3e1./5.0e2)).*-2.0+t58.*t74.*(t19.*(t10.*t24-t4.*t20.*t27).*(6.3e1./5.0e2)+t11.*t27.*(2.0./5.0)+t11.*t14.*t27.*(6.3e1./5.0e2)).*2.0+t57.*t75.*(t19.*(t24.*t35-t4.*t20.*t44).*(6.3e1./5.0e2)+t11.*t44.*(2.0./5.0)+t11.*t14.*t44.*(6.3e1./5.0e2)).*2.0).*(1.0./2.0),t80.*(t58.*t74.*(t4.*t10.*(2.0./5.0)+t14.*t22.*(6.3e1./5.0e2)-t2.*t3.*t11.*(2.0./5.0)+t17.*t19.*t20.*(6.3e1./5.0e2)).*2.0+t57.*t75.*(t4.*t35.*(2.0./5.0)+t14.*t41.*(6.3e1./5.0e2)+t3.*t5.*t11.*(2.0./5.0)+t19.*t20.*t38.*(6.3e1./5.0e2)).*2.0-t56.*t81.*(t8.*t11.*(2.0./5.0)+t14.*t63.*(6.3e1./5.0e2)-t3.*t4.*t9.*(2.0./5.0)-t19.*t20.*t70.*(6.3e1./5.0e2)).*2.0).*(1.0./2.0),t80.*(t19.*t56.*t81.*(t24.*t63-t3.*t6.*t20).*(6.3e1./2.5e2)-t19.*t58.*t74.*(t22.*t24-t20.*t27).*(6.3e1./2.5e2)+t19.*t57.*t75.*(t20.*t44-t24.*t41).*(6.3e1./2.5e2)).*(-1.0./2.0),t80.*(t58.*t74.*(t17.*t19.*(6.3e1./5.0e2)+t14.*t29.*(6.3e1./5.0e2)).*2.0+t57.*t75.*(t19.*t38.*(6.3e1./5.0e2)+t14.*t46.*(6.3e1./5.0e2)).*2.0-t56.*t81.*(t14.*t66.*(6.3e1./5.0e2)-t19.*t70.*(6.3e1./5.0e2)).*2.0).*(-1.0./2.0),0.0];