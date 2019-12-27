function X_ef_0 = FK_UR10(u)

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

T_6_0(1,1)=cos(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*sin(q6);
T_6_0(1,2)=sin(q2 + q3 + q4)*cos(q1)*cos(q6) - sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5));
T_6_0(1,3)=cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5);
T_6_0(1,4)=L4*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L2*sin(q1) - L11*sin(q2 + q3 + q4)*cos(q1);
T_6_0(2,1)=sin(q2 + q3 + q4)*sin(q1)*sin(q6) - cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1));
T_6_0(2,2)=sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) + sin(q2 + q3 + q4)*cos(q6)*sin(q1);
T_6_0(2,3)=cos(q2 + q3 + q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5);
T_6_0(2,4)=- L4*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - L2*cos(q1) - sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) - L11*sin(q2 + q3 + q4)*sin(q1);
T_6_0(3,1)=- cos(q2 + q3 + q4)*sin(q6) - sin(q2 + q3 + q4)*cos(q5)*cos(q6);
T_6_0(3,2)=sin(q2 + q3 + q4)*cos(q5)*sin(q6) - cos(q2 + q3 + q4)*cos(q6);
T_6_0(3,3)=sin(q2 + q3 + q4)*sin(q5);
T_6_0(3,4)=L1 + L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L4*sin(q2 + q3 + q4)*sin(q5);
T_6_0(4,1)=0;
T_6_0(4,2)=0;
T_6_0(4,3)=0;
T_6_0(4,4)=1;

phi_6_0 = angle(-(cos(q1) + sin(q1)*1i)*(cos(q6)*sin(q5)*1i - sin(q2 + q3 + q4)*sin(q6) + cos(q2 + q3 + q4)*cos(q5)*cos(q6)));
theta_6_0 = atan2(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6), (1 - (cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))^2)^(1/2));
psi_6_0 = atan2(sin(q2 + q3 + q4)*cos(q5)*sin(q6) - cos(q2 + q3 + q4)*cos(q6), sin(q2 + q3 + q4)*sin(q5));

X_ef_0 = [T_6_0(1,4);T_6_0(2,4);T_6_0(3,4);psi_6_0;theta_6_0;phi_6_0];

end 
