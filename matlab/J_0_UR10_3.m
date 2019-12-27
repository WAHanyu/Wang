function [J_ef_0,J_cm1_0,J_cm2_0,J_cm3_0] = J_0_UR10_3(u)

% This file was generated automatically by "symModel_UR10_3".

% Input vector: u=[q1,q2,q3,L1,L3,L5,L6,L7,L8L9,,L10,L11].

q1=u(1);
q2=u(2);
q3=u(3);
L1=u(4);
L3=u(5);
L5=u(6);
L6=u(7);
L7=u(8);
L8=u(9);
L9=u(10);
L10=u(11);
L11=u(12);

J_ef_0(1,1)=L4*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L2*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L11*sin(q2 + q3 + q4)*sin(q1);
J_ef_0(1,2)=-cos(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
J_ef_0(1,3)=-cos(q1)*(L5*cos(q2 + q3) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
J_ef_0(2,1)=L4*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L2*sin(q1) - L11*sin(q2 + q3 + q4)*cos(q1);
J_ef_0(2,2)=-sin(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
J_ef_0(2,3)=-sin(q1)*(L5*cos(q2 + q3) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
J_ef_0(3,1)=0;
J_ef_0(3,2)=(L4*sin(q2 + q3 + q4 + q5))/2 - L5*sin(q2 + q3) - L3*sin(q2) - (L4*sin(q2 + q3 + q4 - q5))/2 - L11*sin(q2 + q3 + q4);
J_ef_0(3,3)=(L4*sin(q2 + q3 + q4 + q5))/2 - L5*sin(q2 + q3) - (L4*sin(q2 + q3 + q4 - q5))/2 - L11*sin(q2 + q3 + q4);
J_ef_0(4,1)=0;
J_ef_0(4,2)=sin(q1);
J_ef_0(4,3)=sin(q1);
J_ef_0(5,1)=0;
J_ef_0(5,2)=-cos(q1);
J_ef_0(5,3)=-cos(q1);
J_ef_0(6,1)=1;
J_ef_0(6,2)=0;
J_ef_0(6,3)=0;

