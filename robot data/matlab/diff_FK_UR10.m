function [XOut] = diff_FK_UR10(u)

% This file was generated automatically by "symModel_UR10".

%% parameters
%Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);
q5=u(5);
q6=u(6);
Q=[q1;q2;q3;q4;q5;q6];

%Kinematic Parameters
L1=u(7);
L2=u(8);
L3=u(9);
L4=u(10);
L5=u(11);
L6=u(12);
L7=u(13);
L8=u(14);
L9=u(15);
L10=u(16);
L11=u(17);
L12=u(18);
L13=u(19);
L14=u(20);

%Jacobian
J_6_0(1,1)=L16*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L4*cos(q1) - sin(q1)*(L3*sin(q2 + q3) + L2*sin(q2)) + L5*sin(q2 + q3 + q4)*sin(q1);
J_6_0(1,2)=cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2) - L5*cos(q2 + q3 + q4) - L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(1,3)=-cos(q1)*(L5*cos(q2 + q3 + q4) - L3*cos(q2 + q3) + L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(1,4)=-cos(q1)*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(1,5)=L16*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - L16*sin(q1)*sin(q5);
J_6_0(1,6)=0;
J_6_0(2,1)=L16*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + cos(q1)*(L3*sin(q2 + q3) + L2*sin(q2)) + L4*sin(q1) - L5*sin(q2 + q3 + q4)*cos(q1);
J_6_0(2,2)=sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2) - L5*cos(q2 + q3 + q4) - L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(2,3)=-sin(q1)*(L5*cos(q2 + q3 + q4) - L3*cos(q2 + q3) + L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(2,4)=-sin(q1)*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5));
J_6_0(2,5)=L16*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1));
J_6_0(2,6)=0;
J_6_0(3,1)=0;
J_6_0(3,2)=(L16*sin(q2 + q3 + q4 + q5))/2 + L3*sin(q2 + q3) + L2*sin(q2) - (L16*sin(q2 + q3 + q4 - q5))/2 - L5*sin(q2 + q3 + q4);
J_6_0(3,3)=(L16*sin(q2 + q3 + q4 + q5))/2 + L3*sin(q2 + q3) - (L16*sin(q2 + q3 + q4 - q5))/2 - L5*sin(q2 + q3 + q4);
J_6_0(3,4)=L16*cos(q2 + q3 + q4)*sin(q5) - L5*sin(q2 + q3 + q4);
J_6_0(3,5)=L16*sin(q2 + q3 + q4)*cos(q5);
J_6_0(3,6)=0;
J_6_0(4,1)=0;
J_6_0(4,2)=sin(q1);
J_6_0(4,3)=sin(q1);
J_6_0(4,4)=sin(q1);
J_6_0(4,5)=-sin(q2 + q3 + q4)*cos(q1);
J_6_0(4,6)=cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5);
J_6_0(5,1)=0;
J_6_0(5,2)=-cos(q1);
J_6_0(5,3)=-cos(q1);
J_6_0(5,4)=-cos(q1);
J_6_0(5,5)=-sin(q2 + q3 + q4)*sin(q1);
J_6_0(5,6)=cos(q2 + q3 + q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5);
J_6_0(6,1)=1;
J_6_0(6,2)=0;
J_6_0(6,3)=0;
J_6_0(6,4)=0;
J_6_0(6,5)=cos(q2 + q3 + q4);
J_6_0(6,6)=sin(q2 + q3 + q4)*sin(q5);

%Derivative of jacobian
pJ_6_0(1,1)=pq4*sin(q1)*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5)) - pq2*sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2) - L5*cos(q2 + q3 + q4) - L16*sin(q2 + q3 + q4)*sin(q5)) - pq1*(L4*sin(q1) + L3*sin(q2 + q3)*cos(q1) + L2*cos(q1)*sin(q2) + L16*cos(q5)*sin(q1) - L5*sin(q2 + q3 + q4)*cos(q1) + L16*cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - L16*pq5*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) + pq3*sin(q1)*(L5*cos(q2 + q3 + q4) - L3*cos(q2 + q3) + L16*sin(q2 + q3 + q4)*sin(q5));
pJ_6_0(1,2)=pq4*cos(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) - pq2*cos(q1)*(L3*sin(q2 + q3) + L2*sin(q2) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - pq1*sin(q1)*(L3*cos(q2 + q3) + L2*cos(q2) - L5*cos(q2 + q3 + q4) - L16*sin(q2 + q3 + q4)*sin(q5)) - pq3*cos(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
pJ_6_0(1,3)=pq4*cos(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) + pq1*sin(q1)*(L5*cos(q2 + q3 + q4) - L3*cos(q2 + q3) + L16*sin(q2 + q3 + q4)*sin(q5)) - pq2*cos(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - pq3*cos(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
pJ_6_0(1,4)=pq2*cos(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) + pq3*cos(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) + pq4*cos(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) + pq1*sin(q1)*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
pJ_6_0(1,5)=- L16*pq1*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - L16*pq5*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - L16*pq2*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - L16*pq3*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - L16*pq4*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
pJ_6_0(1,6)=0;
pJ_6_0(2,1)=pq1*(L16*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L4*cos(q1) - sin(q1)*(L3*sin(q2 + q3) + L2*sin(q2)) + L5*sin(q2 + q3 + q4)*sin(q1)) - pq4*(L5*cos(q2 + q3 + q4)*cos(q1) + L16*sin(q2 + q3 + q4)*cos(q1)*sin(q5)) - pq3*(L5*cos(q2 + q3 + q4)*cos(q1) - L3*cos(q2 + q3)*cos(q1) + L16*sin(q2 + q3 + q4)*cos(q1)*sin(q5)) - pq2*(L5*cos(q2 + q3 + q4)*cos(q1) - cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2)) + L16*sin(q2 + q3 + q4)*cos(q1)*sin(q5)) - L16*pq5*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5));
pJ_6_0(2,2)=pq1*cos(q1)*(L3*cos(q2 + q3) + L2*cos(q2) - L5*cos(q2 + q3 + q4) - L16*sin(q2 + q3 + q4)*sin(q5)) - pq2*sin(q1)*(L3*sin(q2 + q3) + L2*sin(q2) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) + pq4*sin(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) - pq3*sin(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
pJ_6_0(2,3)=pq4*sin(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) - pq1*cos(q1)*(L5*cos(q2 + q3 + q4) - L3*cos(q2 + q3) + L16*sin(q2 + q3 + q4)*sin(q5)) - pq2*sin(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - pq3*sin(q1)*(L3*sin(q2 + q3) - L5*sin(q2 + q3 + q4) + L16*cos(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
pJ_6_0(2,4)=pq2*sin(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) - pq1*cos(q1)*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5)) + pq3*sin(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) + pq4*sin(q1)*(L5*sin(q2 + q3 + q4) - L16*cos(q2 + q3 + q4)*sin(q5)) - L16*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
pJ_6_0(2,5)=L16*pq5*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - L16*pq1*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - L16*pq2*sin(q2 + q3 + q4)*cos(q5)*sin(q1) - L16*pq3*sin(q2 + q3 + q4)*cos(q5)*sin(q1) - L16*pq4*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
pJ_6_0(2,6)=0;
pJ_6_0(3,1)=0;
pJ_6_0(3,2)=pq3*((L16*cos(q2 + q3 + q4 + q5))/2 + L3*cos(q2 + q3) - (L16*cos(q2 + q3 + q4 - q5))/2 - L5*cos(q2 + q3 + q4)) + pq2*((L16*cos(q2 + q3 + q4 + q5))/2 + L3*cos(q2 + q3) + L2*cos(q2) - (L16*cos(q2 + q3 + q4 - q5))/2 - L5*cos(q2 + q3 + q4)) - pq4*((L16*cos(q2 + q3 + q4 - q5))/2 - (L16*cos(q2 + q3 + q4 + q5))/2 + L5*cos(q2 + q3 + q4)) + (L16*pq5*(cos(q2 + q3 + q4 + q5) + cos(q2 + q3 + q4 - q5)))/2;
pJ_6_0(3,3)=pq2*((L16*cos(q2 + q3 + q4 + q5))/2 + L3*cos(q2 + q3) - (L16*cos(q2 + q3 + q4 - q5))/2 - L5*cos(q2 + q3 + q4)) + pq3*((L16*cos(q2 + q3 + q4 + q5))/2 + L3*cos(q2 + q3) - (L16*cos(q2 + q3 + q4 - q5))/2 - L5*cos(q2 + q3 + q4)) - pq4*((L16*cos(q2 + q3 + q4 - q5))/2 - (L16*cos(q2 + q3 + q4 + q5))/2 + L5*cos(q2 + q3 + q4)) + (L16*pq5*(cos(q2 + q3 + q4 + q5) + cos(q2 + q3 + q4 - q5)))/2;
pJ_6_0(3,4)=L16*pq5*cos(q2 + q3 + q4)*cos(q5) - pq3*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5)) - pq4*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5)) - pq2*(L5*cos(q2 + q3 + q4) + L16*sin(q2 + q3 + q4)*sin(q5));
pJ_6_0(3,5)=L16*pq2*cos(q2 + q3 + q4)*cos(q5) + L16*pq3*cos(q2 + q3 + q4)*cos(q5) + L16*pq4*cos(q2 + q3 + q4)*cos(q5) - L16*pq5*sin(q2 + q3 + q4)*sin(q5);
pJ_6_0(3,6)=0;
pJ_6_0(4,1)=0;
pJ_6_0(4,2)=pq1*cos(q1);
pJ_6_0(4,3)=pq1*cos(q1);
pJ_6_0(4,4)=pq1*cos(q1);
pJ_6_0(4,5)=pq1*sin(q2 + q3 + q4)*sin(q1) - pq3*cos(q2 + q3 + q4)*cos(q1) - pq4*cos(q2 + q3 + q4)*cos(q1) - pq2*cos(q2 + q3 + q4)*cos(q1);
pJ_6_0(4,6)=pq1*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - pq5*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - pq2*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - pq3*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - pq4*sin(q2 + q3 + q4)*cos(q1)*sin(q5);
pJ_6_0(5,1)=0;
pJ_6_0(5,2)=pq1*sin(q1);
pJ_6_0(5,3)=pq1*sin(q1);
pJ_6_0(5,4)=pq1*sin(q1);
pJ_6_0(5,5)=- pq1*sin(q2 + q3 + q4)*cos(q1) - pq2*cos(q2 + q3 + q4)*sin(q1) - pq3*cos(q2 + q3 + q4)*sin(q1) - pq4*cos(q2 + q3 + q4)*sin(q1);
pJ_6_0(5,6)=pq1*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + pq5*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - pq2*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - pq3*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - pq4*sin(q2 + q3 + q4)*sin(q1)*sin(q5);
pJ_6_0(6,1)=0;
pJ_6_0(6,2)=0;
pJ_6_0(6,3)=0;
pJ_6_0(6,4)=0;
pJ_6_0(6,5)=-sin(q2 + q3 + q4)*(pq2 + pq3 + pq4);
pJ_6_0(6,6)=pq2*cos(q2 + q3 + q4)*sin(q5) + pq3*cos(q2 + q3 + q4)*sin(q5) + pq4*cos(q2 + q3 + q4)*sin(q5) + pq5*sin(q2 + q3 + q4)*cos(q5);

%Output
XOut=[J_6_0,pJ_6_0];
end 
