function [XOut] = Forward_Diff_Kinematics_UR10(u)

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

%Joint Velocity
pq1=u(7);
pq2=u(8);
pq3=u(9);
pq4=u(10);
pq5=u(11);
pq6=u(12);
pQ=[pq1;pq2;pq3;pq4;pq5;pq6];

%Joint Acceleration
ppq1=u(13);
ppq2=u(14);
ppq3=u(15);
ppq4=u(16);
ppq5=u(17);
ppq6=u(18);
ppQ=[ppq1;ppq2;ppq3;ppq4;ppq5;ppq6];

%Kinematic Parameters
L1=u(19);
L2=u(20);
L3=u(21);
L4=u(22);
L5=u(23);
L6=u(24);
L7=u(25);
L8=u(26);
L9=u(27);
L10=u(28);
L11=u(29);
L12=u(30);
L13=u(31);
L14=u(32);

%End-effector position (forward kinematics)
X_ef_0(1,1)=0;
X_ef_0(2,1)=0;
X_ef_0(3,1)=0;
X_ef_0(4,1)=(pi*sign(sin(q2 + q3)))/2;
X_ef_0(5,1)=atan2(cos(q2 + q3), (1 - cos(q2 + q3)^2)^(1/2));
X_ef_0(6,1)=angle(sin(q2 + q3)*(cos(q1) + sin(q1)*1i));

%End-effector velocity (differential kinematics)
pX_ef_0(1,1)=pq1*(L4*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L2*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L11*sin(q2 + q3 + q4)*sin(q1)) - pq5*(L4*sin(q1)*sin(q5) - L4*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - pq2*cos(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5)) - pq4*cos(q1)*(L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5)) - pq3*cos(q1)*(L5*cos(q2 + q3) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
pX_ef_0(2,1)=pq1*(L4*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L2*sin(q1) - L11*sin(q2 + q3 + q4)*cos(q1)) - pq2*sin(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5)) - pq4*sin(q1)*(L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5)) + L4*pq5*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - pq3*sin(q1)*(L5*cos(q2 + q3) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5));
pX_ef_0(3,1)=L4*pq5*sin(q2 + q3 + q4)*cos(q5) - pq4*(L11*sin(q2 + q3 + q4) - L4*cos(q2 + q3 + q4)*sin(q5)) - pq2*(L5*sin(q2 + q3) - (L4*sin(q2 + q3 + q4 + q5))/2 + L3*sin(q2) + (L4*sin(q2 + q3 + q4 - q5))/2 + L11*sin(q2 + q3 + q4)) - pq3*(L5*sin(q2 + q3) - (L4*sin(q2 + q3 + q4 + q5))/2 + (L4*sin(q2 + q3 + q4 - q5))/2 + L11*sin(q2 + q3 + q4));
pX_ef_0(4,1)=pq6*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) + pq2*sin(q1) + pq3*sin(q1) + pq4*sin(q1) - pq5*sin(q2 + q3 + q4)*cos(q1);
pX_ef_0(5,1)=- pq6*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - pq2*cos(q1) - pq3*cos(q1) - pq4*cos(q1) - pq5*sin(q2 + q3 + q4)*sin(q1);
pX_ef_0(6,1)=pq1 + pq5*cos(q2 + q3 + q4) + pq6*sin(q2 + q3 + q4)*sin(q5);

%End-effector acceleration
ppX_ef_0(1,1)=L2*ppq1*cos(q1) - L2*pq1^2*sin(q1) + L5*pq1^2*sin(q2 + q3)*cos(q1) + L5*pq2^2*sin(q2 + q3)*cos(q1) + L5*pq3^2*sin(q2 + q3)*cos(q1) - L11*ppq2*cos(q2 + q3 + q4)*cos(q1) - L11*ppq3*cos(q2 + q3 + q4)*cos(q1) - L11*ppq4*cos(q2 + q3 + q4)*cos(q1) + L11*ppq1*sin(q2 + q3 + q4)*sin(q1) + L3*pq1^2*cos(q1)*sin(q2) + L3*pq2^2*cos(q1)*sin(q2) - L4*pq1^2*cos(q5)*sin(q1) - L4*pq5^2*cos(q5)*sin(q1) + L11*pq1^2*sin(q2 + q3 + q4)*cos(q1) + L11*pq2^2*sin(q2 + q3 + q4)*cos(q1) + L11*pq3^2*sin(q2 + q3 + q4)*cos(q1) + L11*pq4^2*sin(q2 + q3 + q4)*cos(q1) - L5*ppq2*cos(q2 + q3)*cos(q1) - L5*ppq3*cos(q2 + q3)*cos(q1) + L5*ppq1*sin(q2 + q3)*sin(q1) - L3*ppq2*cos(q1)*cos(q2) + L4*ppq1*cos(q1)*cos(q5) + L3*ppq1*sin(q1)*sin(q2) - L4*ppq5*sin(q1)*sin(q5) + 2*L5*pq1*pq2*cos(q2 + q3)*sin(q1) + 2*L5*pq1*pq3*cos(q2 + q3)*sin(q1) + 2*L5*pq2*pq3*sin(q2 + q3)*cos(q1) + 2*L3*pq1*pq2*cos(q2)*sin(q1) - 2*L4*pq1*pq5*cos(q1)*sin(q5) - L4*pq1^2*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*pq2^2*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*pq3^2*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*pq4^2*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*pq5^2*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + 2*L11*pq1*pq2*cos(q2 + q3 + q4)*sin(q1) + 2*L11*pq1*pq3*cos(q2 + q3 + q4)*sin(q1) + 2*L11*pq1*pq4*cos(q2 + q3 + q4)*sin(q1) + 2*L11*pq2*pq3*sin(q2 + q3 + q4)*cos(q1) + 2*L11*pq2*pq4*sin(q2 + q3 + q4)*cos(q1) + 2*L11*pq3*pq4*sin(q2 + q3 + q4)*cos(q1) + L4*ppq5*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - L4*ppq1*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*ppq2*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*ppq3*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - L4*ppq4*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq2*pq3*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq1*pq5*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - 2*L4*pq2*pq4*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq2*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - 2*L4*pq3*pq4*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq3*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5) - 2*L4*pq4*pq5*sin(q2 + q3 + q4)*cos(q1)*cos(q5) + 2*L4*pq1*pq2*sin(q2 + q3 + q4)*sin(q1)*sin(q5) + 2*L4*pq1*pq3*sin(q2 + q3 + q4)*sin(q1)*sin(q5) + 2*L4*pq1*pq4*sin(q2 + q3 + q4)*sin(q1)*sin(q5);
ppX_ef_0(2,1)=L2*ppq1*sin(q1) + L2*pq1^2*cos(q1) + L5*pq1^2*sin(q2 + q3)*sin(q1) + L5*pq2^2*sin(q2 + q3)*sin(q1) + L5*pq3^2*sin(q2 + q3)*sin(q1) - L11*ppq1*sin(q2 + q3 + q4)*cos(q1) - L11*ppq2*cos(q2 + q3 + q4)*sin(q1) - L11*ppq3*cos(q2 + q3 + q4)*sin(q1) - L11*ppq4*cos(q2 + q3 + q4)*sin(q1) + L4*pq1^2*cos(q1)*cos(q5) + L4*pq5^2*cos(q1)*cos(q5) + L3*pq1^2*sin(q1)*sin(q2) + L3*pq2^2*sin(q1)*sin(q2) + L11*pq1^2*sin(q2 + q3 + q4)*sin(q1) + L11*pq2^2*sin(q2 + q3 + q4)*sin(q1) + L11*pq3^2*sin(q2 + q3 + q4)*sin(q1) + L11*pq4^2*sin(q2 + q3 + q4)*sin(q1) - L5*ppq1*sin(q2 + q3)*cos(q1) - L5*ppq2*cos(q2 + q3)*sin(q1) - L5*ppq3*cos(q2 + q3)*sin(q1) - L3*ppq1*cos(q1)*sin(q2) - L3*ppq2*cos(q2)*sin(q1) + L4*ppq1*cos(q5)*sin(q1) + L4*ppq5*cos(q1)*sin(q5) - 2*L5*pq1*pq2*cos(q2 + q3)*cos(q1) - 2*L5*pq1*pq3*cos(q2 + q3)*cos(q1) - L4*ppq2*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*ppq3*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*ppq4*sin(q2 + q3 + q4)*sin(q1)*sin(q5) + 2*L5*pq2*pq3*sin(q2 + q3)*sin(q1) - 2*L3*pq1*pq2*cos(q1)*cos(q2) - 2*L4*pq1*pq5*sin(q1)*sin(q5) - L4*pq1^2*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*pq2^2*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*pq3^2*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*pq4^2*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - L4*pq5^2*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 2*L11*pq1*pq2*cos(q2 + q3 + q4)*cos(q1) - 2*L11*pq1*pq3*cos(q2 + q3 + q4)*cos(q1) - 2*L11*pq1*pq4*cos(q2 + q3 + q4)*cos(q1) + 2*L11*pq2*pq3*sin(q2 + q3 + q4)*sin(q1) + 2*L11*pq2*pq4*sin(q2 + q3 + q4)*sin(q1) + 2*L11*pq3*pq4*sin(q2 + q3 + q4)*sin(q1) + L4*ppq1*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + L4*ppq5*cos(q2 + q3 + q4)*cos(q5)*sin(q1) + 2*L4*pq1*pq5*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 2*L4*pq1*pq2*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq1*pq3*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq1*pq4*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - 2*L4*pq2*pq3*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 2*L4*pq2*pq4*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 2*L4*pq2*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1) - 2*L4*pq3*pq4*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - 2*L4*pq3*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1) - 2*L4*pq4*pq5*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
ppX_ef_0(3,1)=(L4*pq2^2*cos(q2 + q3 + q4 + q5))/2 - L3*ppq2*sin(q2) + (L4*pq3^2*cos(q2 + q3 + q4 + q5))/2 + (L4*pq4^2*cos(q2 + q3 + q4 + q5))/2 + (L4*pq5^2*cos(q2 + q3 + q4 + q5))/2 - (L4*ppq2*sin(q2 + q3 + q4 - q5))/2 - (L4*ppq3*sin(q2 + q3 + q4 - q5))/2 - (L4*ppq4*sin(q2 + q3 + q4 - q5))/2 + (L4*ppq5*sin(q2 + q3 + q4 - q5))/2 - L5*pq2^2*cos(q2 + q3) - L5*pq3^2*cos(q2 + q3) - L11*ppq2*sin(q2 + q3 + q4) - L11*ppq3*sin(q2 + q3 + q4) - L11*ppq4*sin(q2 + q3 + q4) - L3*pq2^2*cos(q2) - (L4*pq2^2*cos(q2 + q3 + q4 - q5))/2 - (L4*pq3^2*cos(q2 + q3 + q4 - q5))/2 - (L4*pq4^2*cos(q2 + q3 + q4 - q5))/2 - (L4*pq5^2*cos(q2 + q3 + q4 - q5))/2 - L11*pq2^2*cos(q2 + q3 + q4) - L11*pq3^2*cos(q2 + q3 + q4) - L11*pq4^2*cos(q2 + q3 + q4) + (L4*ppq2*sin(q2 + q3 + q4 + q5))/2 + (L4*ppq3*sin(q2 + q3 + q4 + q5))/2 + (L4*ppq4*sin(q2 + q3 + q4 + q5))/2 + (L4*ppq5*sin(q2 + q3 + q4 + q5))/2 - L5*ppq2*sin(q2 + q3) - L5*ppq3*sin(q2 + q3) + L4*pq2*pq3*cos(q2 + q3 + q4 + q5) + L4*pq2*pq4*cos(q2 + q3 + q4 + q5) + L4*pq2*pq5*cos(q2 + q3 + q4 + q5) + L4*pq3*pq4*cos(q2 + q3 + q4 + q5) + L4*pq3*pq5*cos(q2 + q3 + q4 + q5) + L4*pq4*pq5*cos(q2 + q3 + q4 + q5) - 2*L5*pq2*pq3*cos(q2 + q3) - L4*pq2*pq3*cos(q2 + q3 + q4 - q5) - L4*pq2*pq4*cos(q2 + q3 + q4 - q5) + L4*pq2*pq5*cos(q2 + q3 + q4 - q5) - L4*pq3*pq4*cos(q2 + q3 + q4 - q5) + L4*pq3*pq5*cos(q2 + q3 + q4 - q5) + L4*pq4*pq5*cos(q2 + q3 + q4 - q5) - 2*L11*pq2*pq3*cos(q2 + q3 + q4) - 2*L11*pq2*pq4*cos(q2 + q3 + q4) - 2*L11*pq3*pq4*cos(q2 + q3 + q4);
ppX_ef_0(4,1)=ppq2*sin(q1) + ppq3*sin(q1) + ppq4*sin(q1) + pq1*pq2*cos(q1) + pq1*pq3*cos(q1) + pq1*pq4*cos(q1) + ppq6*cos(q5)*sin(q1) - ppq5*sin(q2 + q3 + q4)*cos(q1) - pq5*pq6*sin(q1)*sin(q5) - pq2*pq5*cos(q2 + q3 + q4)*cos(q1) - pq3*pq5*cos(q2 + q3 + q4)*cos(q1) - pq4*pq5*cos(q2 + q3 + q4)*cos(q1) + pq1*pq5*sin(q2 + q3 + q4)*sin(q1) + ppq6*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + pq1*pq6*cos(q1)*cos(q5) - pq1*pq6*cos(q2 + q3 + q4)*sin(q1)*sin(q5) - pq2*pq6*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - pq3*pq6*sin(q2 + q3 + q4)*cos(q1)*sin(q5) - pq4*pq6*sin(q2 + q3 + q4)*cos(q1)*sin(q5) + pq5*pq6*cos(q2 + q3 + q4)*cos(q1)*cos(q5);
ppX_ef_0(5,1)=pq1*pq2*sin(q1) - ppq3*cos(q1) - ppq4*cos(q1) - ppq2*cos(q1) + pq1*pq3*sin(q1) + pq1*pq4*sin(q1) - ppq6*cos(q1)*cos(q5) - ppq5*sin(q2 + q3 + q4)*sin(q1) - pq1*pq5*sin(q2 + q3 + q4)*cos(q1) - pq2*pq5*cos(q2 + q3 + q4)*sin(q1) - pq3*pq5*cos(q2 + q3 + q4)*sin(q1) - pq4*pq5*cos(q2 + q3 + q4)*sin(q1) + ppq6*cos(q2 + q3 + q4)*sin(q1)*sin(q5) + pq1*pq6*cos(q5)*sin(q1) + pq5*pq6*cos(q1)*sin(q5) + pq1*pq6*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + pq5*pq6*cos(q2 + q3 + q4)*cos(q5)*sin(q1) - pq2*pq6*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - pq3*pq6*sin(q2 + q3 + q4)*sin(q1)*sin(q5) - pq4*pq6*sin(q2 + q3 + q4)*sin(q1)*sin(q5);
ppX_ef_0(6,1)=ppq1 + pq6*(pq2*cos(q2 + q3 + q4)*sin(q5) + pq3*cos(q2 + q3 + q4)*sin(q5) + pq4*cos(q2 + q3 + q4)*sin(q5) + pq5*sin(q2 + q3 + q4)*cos(q5)) + ppq5*cos(q2 + q3 + q4) - pq5*sin(q2 + q3 + q4)*(pq2 + pq3 + pq4) + ppq6*sin(q2 + q3 + q4)*sin(q5);

%Output
XOut=[X_ef_0,pX_ef_0,ppX_ef_0];
end 
