function symModel_UR10()
% Date: 12/24/2019
% Author: Maximilian Bader
% Description:
%   Script function that creates a model of the robot as an own MATLAB
%   function (model includes kinematics & dynamics).
%   Script function is based on the Universal Robot UR10. The DH table is
%   copied from the manufacturer's documentation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
%close all;
clc;

% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 pq1 pq2 pq3 pq4 pq5 pq6 ppq1 ppq2 ppq3 ppq1 ppq2 ...
     ppq3 ppq4 ppq5 ppq6 L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15 ...
     L16 m1 m2 m3 m4 m5 m6 g gx gy gz k1 k2 k3 k4 k5 k6 beta1 beta2 beta3...
     beta4 beta5 beta6 I111 I112 I113 I122 I123 I133 I211 I212 I213 I222 ...
     I223 I233 I311 I312 I313 I322 I323 I333 I411 I412 I413 I422 I423 I433 ...
     I511 I512 I513 I522 I523 I533 I611 I612 I613 I622 I623 I633 pq1r pq2r ...
     pq3r pq4r pq5r pq6r ppq1r ppq2r ppq3r ppq4r ppq5r ppq6r t_final t_init real;

%% Robot Kinematics
% D-H table from geometric inspection
Dh = [q1       L1   0    pi/2;      % coordinate frame 1
      q2-pi/2  0    -L3  0;         % coordinate frame 2
      q3       0    -L5  0;         % coordinate frame 3
      q4-pi/2  L2   0    pi/2;      % coordinate frame 4    
      q5       L11  0    -pi/2;     % coordinate frame 5
      q6       L4   0    0;         % coordinate frame 6
      q1       L6   0    0;         % center of mass 1
      q2-pi/2  L7   -L8  0;         % center of mass 2
      q3       L9   -L10 0;         % center of mass 3
      q4-pi/2  L12  0    0;         % center of mass 4
      q5       L13  0    0;         % center of mass 5    
      q6       L14  0    0;];       % center of mass 6

sDh=size(Dh);

% compute relative transformations
for i=1:sDh(1)
    Rz= [cos(Dh(i,1)) -sin(Dh(i,1)) 0   0   ;
        sin(Dh(i,1))  cos(Dh(i,1))  0   0   ;
        0             0             1   0   ;
        0             0             0   1   ];
    
    Tz= [1            0       0   0   ;
        0             1       0   0   ;
        0             0       1   Dh(i,2);
        0             0       0   1   ];
    
    Tx= [1            0       0   Dh(i,3);
        0             1       0   0   ;
        0             0       1   0   ;
        0             0       0   1   ];
    
    Rx= [1     0             0        0   ;
        0 cos(Dh(i,4)) -sin(Dh(i,4))  0   ;
        0 sin(Dh(i,4))  cos(Dh(i,4))  0   ;
        0      0             0        1   ];
    
    % compute relative homogeneous transformation for each frame
    relHt{i}=simplify(Rz*Tz*Tx*Rx);
end
  
% Homogeneous transformations
T_i_0{1} = relHt{1};
T_i_0{2} = simplify(T_i_0{1}*relHt{2});
T_i_0{3} = simplify(T_i_0{2}*relHt{3});
T_i_0{4} = simplify(T_i_0{3}*relHt{4});
T_i_0{5} = simplify(T_i_0{4}*relHt{5});
T_i_0{6} = simplify(T_i_0{5}*relHt{6});
T_i_0{7} = relHt{7};
T_i_0{8} = simplify(T_i_0{1}*relHt{8});
T_i_0{9} = simplify(T_i_0{2}*relHt{9});
T_i_0{10} = simplify(T_i_0{3}*relHt{10});
T_i_0{11} = simplify(T_i_0{4}*relHt{11});
T_i_0{12} = simplify(T_i_0{5}*relHt{12});

% Euler angles of end-effector
%d = sqrt(1-T_i_0{6}(3,1)^2);
phi_EF_0=simplify(atan2(T_i_0{6}(2,1),T_i_0{6}(1,1))); % yaw around x-axis
theta_EF_0=simplify(atan2(-T_i_0{6}(3,1),sqrt(1-T_i_0{6}(3,1)^2))); % pitch around y-axis
psi_EF_0=simplify(atan2(T_i_0{6}(3,2),T_i_0{6}(3,3))); % roll arount z-axis

% Forward kinematics for end-effector
X_ef_0 = [T_i_0{6}(1,4);T_i_0{6}(2,4);T_i_0{6}(3,4);psi_EF_0;theta_EF_0;phi_EF_0];
  
%% Differential kinematics
% Jacobian matrix for end-effector
% Explanation: t_6_0 = T_i_0{6}(1:3,4), z_0 = [0;0;1], z_1 =
%              T_i_0{1}(1:3,3), ...
J_ef_0 = sym(zeros(6,6));
J_ef_0(1:3,1) = simplify(cross([0;0;1],T_i_0{6}(1:3,4)));
J_ef_0(4:6,1) = [0;0;1];
J_ef_0(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{6}(1:3,4)-T_i_0{1}(1:3,4))));
J_ef_0(4:6,2) = T_i_0{1}(1:3,3);
J_ef_0(1:3,3) = simplify(cross(T_i_0{2}(1:3,3),(T_i_0{6}(1:3,4)-T_i_0{2}(1:3,4))));
J_ef_0(4:6,3) = T_i_0{2}(1:3,3);
J_ef_0(1:3,4) = simplify(cross(T_i_0{3}(1:3,3),(T_i_0{6}(1:3,4)-T_i_0{3}(1:3,4))));
J_ef_0(4:6,4) = T_i_0{3}(1:3,3);
J_ef_0(1:3,5) = simplify(cross(T_i_0{4}(1:3,3),(T_i_0{6}(1:3,4)-T_i_0{4}(1:3,4))));
J_ef_0(4:6,5) = T_i_0{4}(1:3,3);
J_ef_0(1:3,6) = simplify(cross(T_i_0{5}(1:3,3),(T_i_0{6}(1:3,4)-T_i_0{5}(1:3,4))));
J_ef_0(4:6,6) = T_i_0{5}(1:3,3);

% derivative of jacobian
pJ_ef_0 = simplify(diff(J_ef_0,q1)*pq1 + diff(J_ef_0,q2)*pq2 + diff(J_ef_0,q3)*pq3 ...
                    + diff(J_ef_0,q4)*pq4 + diff(J_ef_0,q5)*pq5 + diff(J_ef_0,q6)*pq6);

% jacobian of center of mass 1
J_cmi_0{1} = sym(zeros(6,6));
J_cmi_0{1}(1:3,1) = simplify(cross([0;0;1],T_i_0{7}(1:3,4)));
J_cmi_0{1}(4:6,1) = [0;0;1];

% jacobian of center of mass 2
J_cmi_0{2} = sym(zeros(6,6));
J_cmi_0{2}(1:3,1) = simplify(cross([0;0;1],T_i_0{8}(1:3,4)));
J_cmi_0{2}(4:6,1) = [0;0;1];
J_cmi_0{2}(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{8}(1:3,4)-T_i_0{1}(1:3,4))));
J_cmi_0{2}(4:6,2) = T_i_0{1}(1:3,3);

% jacobian of center of mass 3
J_cmi_0{3} = sym(zeros(6,6));
J_cmi_0{3}(1:3,1) = simplify(cross([0;0;1],T_i_0{9}(1:3,4)));
J_cmi_0{3}(4:6,1) = [0;0;1];
J_cmi_0{3}(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{9}(1:3,4)-T_i_0{1}(1:3,4))));
J_cmi_0{3}(4:6,2) = T_i_0{1}(1:3,3);
J_cmi_0{3}(1:3,3) = simplify(cross(T_i_0{2}(1:3,3),(T_i_0{9}(1:3,4)-T_i_0{2}(1:3,4))));
J_cmi_0{3}(4:6,3) = T_i_0{2}(1:3,3);

% jacobian of center of mass 4
J_cmi_0{4} = sym(zeros(6,6));
J_cmi_0{4}(1:3,1) = simplify(cross([0;0;1],T_i_0{10}(1:3,4)));
J_cmi_0{4}(4:6,1) = [0;0;1];
J_cmi_0{4}(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{10}(1:3,4)-T_i_0{1}(1:3,4))));
J_cmi_0{4}(4:6,2) = T_i_0{1}(1:3,3);
J_cmi_0{4}(1:3,3) = simplify(cross(T_i_0{2}(1:3,3),(T_i_0{10}(1:3,4)-T_i_0{2}(1:3,4))));
J_cmi_0{4}(4:6,3) = T_i_0{2}(1:3,3);
J_cmi_0{4}(1:3,4) = simplify(cross(T_i_0{3}(1:3,3),(T_i_0{10}(1:3,4)-T_i_0{3}(1:3,4))));
J_cmi_0{4}(4:6,4) = T_i_0{3}(1:3,3);

% jacobian of center of mass 5
J_cmi_0{5} = sym(zeros(6,6));
J_cmi_0{5}(1:3,1) = simplify(cross([0;0;1],T_i_0{11}(1:3,4)));
J_cmi_0{5}(4:6,1) = [0;0;1];
J_cmi_0{5}(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{11}(1:3,4)-T_i_0{1}(1:3,4))));
J_cmi_0{5}(4:6,2) = T_i_0{1}(1:3,3);
J_cmi_0{5}(1:3,3) = simplify(cross(T_i_0{2}(1:3,3),(T_i_0{11}(1:3,4)-T_i_0{2}(1:3,4))));
J_cmi_0{5}(4:6,3) = T_i_0{2}(1:3,3);
J_cmi_0{5}(1:3,4) = simplify(cross(T_i_0{3}(1:3,3),(T_i_0{11}(1:3,4)-T_i_0{3}(1:3,4))));
J_cmi_0{5}(4:6,4) = T_i_0{3}(1:3,3);
J_cmi_0{5}(1:3,5) = simplify(cross(T_i_0{4}(1:3,3),(T_i_0{11}(1:3,4)-T_i_0{4}(1:3,4))));
J_cmi_0{5}(4:6,5) = T_i_0{4}(1:3,3);

% jacobian of center of mass 6
J_cmi_0{6} = sym(zeros(6,6));
J_cmi_0{6}(1:3,1) = simplify(cross([0;0;1],T_i_0{12}(1:3,4)));
J_cmi_0{6}(4:6,1) = [0;0;1];
J_cmi_0{6}(1:3,2) = simplify(cross(T_i_0{1}(1:3,3),(T_i_0{12}(1:3,4)-T_i_0{1}(1:3,4))));
J_cmi_0{6}(4:6,2) = T_i_0{1}(1:3,3);
J_cmi_0{6}(1:3,3) = simplify(cross(T_i_0{2}(1:3,3),(T_i_0{12}(1:3,4)-T_i_0{2}(1:3,4))));
J_cmi_0{6}(4:6,3) = T_i_0{2}(1:3,3);
J_cmi_0{6}(1:3,4) = simplify(cross(T_i_0{3}(1:3,3),(T_i_0{12}(1:3,4)-T_i_0{3}(1:3,4))));
J_cmi_0{6}(4:6,4) = T_i_0{3}(1:3,3);
J_cmi_0{6}(1:3,5) = simplify(cross(T_i_0{4}(1:3,3),(T_i_0{12}(1:3,4)-T_i_0{4}(1:3,4))));
J_cmi_0{6}(4:6,5) = T_i_0{4}(1:3,3);
J_cmi_0{6}(1:3,6) = simplify(cross(T_i_0{5}(1:3,3),(T_i_0{12}(1:3,4)-T_i_0{5}(1:3,4))));
J_cmi_0{6}(4:6,6) = T_i_0{5}(1:3,3);

%% Trajectory Planning based on Jacobian
% Matrix of 5th degree polynomial 
T = [1 0                0                   0                       0                       0;
     1 (t_final-t_init) (t_final-t_init)^2  (t_final-t_init)^3      (t_final-t_init)^4      (t_final-t_init)^5;
     0 1                0                   0                       0                       0;
     0 1                2*(t_final-t_init)  3*(t_final-t_init)^2    4*(t_final-t_init)^3    5*(t_final-t_init)^4;
     0 0                2                   0                       0                       0;
     0 0                2                   6*(t_final-t_init)      12*(t_final-t_init)^2   20*(t_final-t_init)^3;];
 
% Solution vector for linear system equation
x = [0; 1; 0; 0; 0; 0];
 
% Solve system of linear equations
a_x = simplify(T\x);

%% Intertia Matrices for centers of mass
% I{1} = [I111 I112 I113 I114 I115 I116;
%         I112 I122 I123 I124 I125 I126;
%         I113 I123 I133 I134 I135 I136;
%         I114 I124 I134 I144 I145 I146;
%         I115 I125 I135 I145 I155 I156;
%         I116 I126 I136 I146 I156 I166];
  
I{1} = [I111 I112 I113;
        I112 I122 I123;
        I113 I123 I133];

I{2} = [I211 I212 I213;
        I212 I222 I223;
        I213 I223 I233];
  
I{3} = [I311 I312 I313;
        I312 I322 I323;
        I313 I323 I333];
  
I{4} = [I411 I412 I413;
        I412 I422 I423;
        I413 I423 I433];
  
I{5} = [I511 I512 I513;
        I512 I522 I523;
        I513 I523 I533];
    
I{6} = [I611 I612 I613;
        I612 I622 I623;
        I613 I623 I633];


%% inertia matrix M
% create cell arrays
Mm=[m1;m2;m3;m4;m5;m6];
J_cmi_0_v = {J_cmi_0{1}(1:3,1:end), J_cmi_0{2}(1:3,1:end), J_cmi_0{3}(1:3,1:end), ...
             J_cmi_0{4}(1:3,1:end), J_cmi_0{5}(1:3,1:end), J_cmi_0{6}(1:3,1:end)};
J_cmi_0_omega = {J_cmi_0{1}(4:6,1:end), J_cmi_0{2}(4:6,1:end), J_cmi_0{3}(4:6,1:end), ...
                 J_cmi_0{4}(4:6,1:end), J_cmi_0{5}(4:6,1:end), J_cmi_0{6}(4:6,1:end)};
R_cmi_0 = {T_i_0{7}(1:3,1:3), T_i_0{8}(1:3,1:3), T_i_0{9}(1:3,1:3), ...
           T_i_0{10}(1:3,1:3), T_i_0{11}(1:3,1:3), T_i_0{12}(1:3,1:3)};

% compute M
% Only simplify the first 5 matrices, because for some weird reason, 
% simplifying the 6th equation gives a non-symmetric equation
% -> Skipping simplify for the 6th matrix
M = sym(zeros(6));
M_unsimplified = sym(zeros(6));
M_i = cell(1,6);
test_M_i = cell(1,6);
for i=1:6
    i
    % Generate matrix for each center of mass
    M_i{i} = Mm(i)*(J_cmi_0_v{i}')*J_cmi_0_v{i} + (J_cmi_0_omega{i}')*R_cmi_0{i}*I{i}*(R_cmi_0{i}')*J_cmi_0_omega{i};
    
    % Sum all inertia matrices
    M_unsimplified = M_unsimplified + M_i{i};
    M = simplify(M + M_i{i});
    
    % test whether each single inertia matrix is symmetric
    test_M_i{i} = simplify(M_i{i}-M_i{i}');    
end

clear i Mm J_cmi_0_v J_cmi_0_omega R_cmi_0;

%% matrix of Coriolis & Centripetal effects
% partial derivatives
dM_dqi{1} = simplify(diff(M,q1));
dM_dqi{2} = simplify(diff(M,q2));
dM_dqi{3} = simplify(diff(M,q3));
dM_dqi{4} = simplify(diff(M,q4));
dM_dqi{5} = simplify(diff(M,q5));
dM_dqi{6} = simplify(diff(M,q6));

% c matrix 
C = sym(zeros(6));
for k=1:6
    for j=1:6
        C(k,j) = simplify(0.5*( (dM_dqi{1}(k,j)+dM_dqi{j}(k,1)-dM_dqi{k}(1,j))*pq1 ...
                        + (dM_dqi{2}(k,j)+dM_dqi{j}(k,2)-dM_dqi{k}(2,j))*pq2 ...  
                        + (dM_dqi{3}(k,j)+dM_dqi{j}(k,3)-dM_dqi{k}(3,j))*pq3 ...
                        + (dM_dqi{4}(k,j)+dM_dqi{j}(k,4)-dM_dqi{k}(4,j))*pq4 ...
                        + (dM_dqi{5}(k,j)+dM_dqi{j}(k,5)-dM_dqi{k}(5,j))*pq5 ...
                        + (dM_dqi{6}(k,j)+dM_dqi{j}(k,6)-dM_dqi{k}(6,j))*pq6));
    end
end

clear k j;

%% gravitational torques vector G
% IMPORTANT: Potential energy increases, if the robot goes up -> negative
%            sign for g, if multiplied with correct g vector
% mass 1
P_cm{1} = simplify(m1*g*[gx,gy,gz]*T_i_0{7}(1:3,4));

% mass 2
P_cm{2} = simplify(m2*g*[gx,gy,gz]*T_i_0{8}(1:3,4));

% mass 3
P_cm{3} = simplify(m3*g*[gx,gy,gz]*T_i_0{9}(1:3,4));

% mass 4
P_cm{4} = simplify(m4*g*[gx,gy,gz]*T_i_0{10}(1:3,4));

% mass 5
P_cm{5} = simplify(m5*g*[gx,gy,gz]*T_i_0{11}(1:3,4));

% mass 6
P_cm{6} = simplify(m6*g*[gx,gy,gz]*T_i_0{12}(1:3,4));

% overall
Pt = simplify(P_cm{1}+P_cm{2}+P_cm{3}+P_cm{4}+P_cm{5}+P_cm{6});

% g vector
G = [simplify(diff(Pt,q1));
     simplify(diff(Pt,q2));
     simplify(diff(Pt,q3));
     simplify(diff(Pt,q4));
     simplify(diff(Pt,q5));
     simplify(diff(Pt,q6));];
 
%% test matrices
test_M = simplify(M_unsimplified-M_unsimplified');
pM = simplify(dM_dqi{1}*pq1+dM_dqi{2}*pq2+dM_dqi{3}*pq3+dM_dqi{4}*pq4 ...
                +dM_dqi{5}*pq5+dM_dqi{6}*pq6);
N = simplify(pM-2*C);
test_N = simplify(N+N');
test_N_2 = simplify([q1,q2,q3,q4,q5,q6]*N*[q1;q2;q3;q4;q5;q6]);

% print results
% fprintf('--------------------------\n');
% fprintf('The results of matrix tests for the motion equations:\n');
% if(isAlways(test_M == zeros(3)))
%    fprintf(' - Inertia matrix M: Fullfills the requirement M=M^T.\n');
% else
%    fprintf(' - Inertia matrix M: Does NOT fulfill the requirement M=M^T.\n');
% end
% if(isAlways(test_N == zeros(3) & test_N_2 == 0))
%    fprintf(' - Centrifugal & coriolis effects matrix N: Fullfills the requirement N=-N^T.\n');
% else
%    fprintf(' - Centrifugal & coriolis effects matrix N: Does NOT fulfill the requirement N=-N^T.\n');
% end
% fprintf('--------------------------\n');

%% Test matrices added to dynamic model
% inertia matrix M
M_dynModel(1,1) = 4*I133 + 2*I222 + I311 + I322 + I311*cos(2*q2 + 2*q3) ...
                    - I322*cos(2*q2 + 2*q3) + 2*cos(q2 + q3)*(I311*cos(q2 + q3) ...
                    - I312*sin(q2 + q3)) + 2*m3*(L9*cos(q1) + L3*sin(q1)*sin(q2) ...
                    + L10*cos(q2)*sin(q1)*sin(q3) + L10*cos(q3)*sin(q1)*sin(q2))^2 ...
                    - 2*I312*sin(2*q2 + 2*q3) - 2*sin(q2 + q3)*(I312*cos(q2 + q3) ...
                    - I322*sin(q2 + q3)) + 4*m5*(cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) ...
                    - L2*sin(q1) + L13*sin(q2 + q3 + q4)*cos(q1))^2 + 4*m5*(L2*cos(q1) ...
                    + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L13*sin(q2 + q3 + q4)*sin(q1))^2 ...
                    + 4*cos(q2 + q3 + q4)*(I422*cos(q2 + q3 + q4) + I412*sin(q2 + q3 + q4)) ...
                    + 4*sin(q2 + q3 + q4)*(I412*cos(q2 + q3 + q4) + I411*sin(q2 + q3 + q4)) ...
                    + L3^2*m3 + 2*L7^2*m2 + 2*L8^2*m2 + 2*L9^2*m3 + L10^2*m3 + (4*cos(q2 + q3 + q4)*sin(q6) ...
                    + 4*sin(q2 + q3 + q4)*cos(q5)*cos(q6))*(I611*(cos(q2 + q3 + q4)*sin(q6) ...
                    + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I612*(cos(q2 + q3 + q4)*cos(q6) ...
                    - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I613*sin(q2 + q3 + q4)*sin(q5)) + (4*cos(q2 + q3 + q4)*cos(q6)...
                    - 4*sin(q2 + q3 + q4)*cos(q5)*sin(q6))*(I612*(cos(q2 + q3 + q4)*sin(q6) ...
                    + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I622*(cos(q2 + q3 + q4)*cos(q6) ...
                    - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I623*sin(q2 + q3 + q4)*sin(q5)) ...
                    + 2*m2*(L7*cos(q1) + L8*sin(q1)*sin(q2))^2 + 2*m2*(L7*sin(q1) - L8*cos(q1)*sin(q2))^2 ...
                    + 4*m6*(L14*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) - cos(q1)*(L5*sin(q2 + q3) ...
                    + L3*sin(q2)) + L2*sin(q1) - L11*sin(q2 + q3 + q4)*cos(q1))^2 + 2*cos(q2)*(I211*cos(q2) ...
                    - I212*sin(q2)) + 4*m4*(L12*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)))^2 ...
                    + 4*m4*(cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) - L12*sin(q1))^2 + 4*m6*(L14*(cos(q1)*cos(q5) ...
                    - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) + L2*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) ...
                    + L11*sin(q2 + q3 + q4)*sin(q1))^2 - 2*sin(q2)*(I212*cos(q2) - I222*sin(q2)) + 2*I211*cos(q2)^2 ...
                    - 2*I222*cos(q2)^2 + 4*cos(q2 + q3 + q4)*(I533*cos(q2 + q3 + q4) - I513*sin(q2 + q3 + q4)*cos(q5) ...
                    + I523*sin(q2 + q3 + q4)*sin(q5)) - 2*I212*sin(2*q2) + 2*m3*(L3*cos(q1)*sin(q2) ...
                    - L9*sin(q1) + L10*cos(q1)*cos(q2)*sin(q3) + L10*cos(q1)*cos(q3)*sin(q2))^2 ...
                    - 4*sin(q2 + q3 + q4)*sin(q5)*(I613*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) ...
                    + I623*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) ...
                    - I633*sin(q2 + q3 + q4)*sin(q5)) - L3^2*m3*cos(2*q2) - 2*L8^2*m2*cos(q2)^2 ...
                    - 4*sin(q2 + q3 + q4)*cos(q5)*(I513*cos(q2 + q3 + q4) - I511*sin(q2 + q3 + q4)*cos(q5) ...
                    + I512*sin(q2 + q3 + q4)*sin(q5)) - L10^2*m3*cos(2*q2 + 2*q3) ...
                    + 4*sin(q2 + q3 + q4)*sin(q5)*(I523*cos(q2 + q3 + q4) - I512*sin(q2 + q3 + q4)*cos(q5) ...
                    + I522*sin(q2 + q3 + q4)*sin(q5)) + 2*L3*L10*m3*cos(q3) - 2*L3*L10*m3*cos(2*q2 + q3);
M_dynModel(1,2) = (I613*sin(q2 + q3 + q4 + q5 - q6))/4 - (I612*sin(q2 + q3 + q4 + q5 + 2*q6))/4 - (I612*sin(q2 + q3 + q4 + q5 - 2*q6))/4 - (I613*sin(q2 + q3 + q4 - q5 + q6))/4 - (I613*sin(q2 + q3 + q4 - 2*q5 + q6))/4 - (I613*sin(q2 + q3 + q4 + 2*q5 + q6))/4 + (I523*cos(q2 + q3 + q4 + q5))/2 + (I513*sin(q2 + q3 + q4 + q5))/2 - I313*cos(q2 + q3) + I323*sin(q2 + q3) + (I611*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (I611*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I611*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (I611*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (I611*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (I611*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (I622*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (I622*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I623*cos(q2 + q3 + q4 - q5 - q6))/4 + (I622*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (I622*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (I622*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (I622*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (I623*cos(q2 + q3 + q4 - 2*q5 - q6))/4 + (I623*cos(q2 + q3 + q4 + 2*q5 - q6))/4 + (I612*sin(q2 + q3 + q4 - q5 - 2*q6))/4 + (I612*sin(q2 + q3 + q4 - q5 + 2*q6))/4 + (I613*sin(q2 + q3 + q4 - q5 - q6))/4 - (I612*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/8 + (I612*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/8 + (I612*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/8 - (I612*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/8 - (I613*sin(q2 + q3 + q4 - 2*q5 - q6))/4 - (I613*sin(q2 + q3 + q4 + 2*q5 - q6))/4 - I213*cos(q2) + I223*sin(q2) - (I511*cos(q2 + q3 + q4 - 2*q5))/4 + (I511*cos(q2 + q3 + q4 + 2*q5))/4 + (I522*cos(q2 + q3 + q4 - 2*q5))/4 - (I522*cos(q2 + q3 + q4 + 2*q5))/4 + (I523*cos(q2 + q3 + q4 - q5))/2 - (I611*cos(q2 + q3 + q4 - 2*q5))/8 + (I611*cos(q2 + q3 + q4 + 2*q5))/8 - (I622*cos(q2 + q3 + q4 - 2*q5))/8 + (I622*cos(q2 + q3 + q4 + 2*q5))/8 + (I633*cos(q2 + q3 + q4 - 2*q5))/4 - (I633*cos(q2 + q3 + q4 + 2*q5))/4 - (I512*sin(q2 + q3 + q4 - 2*q5))/2 - (I512*sin(q2 + q3 + q4 + 2*q5))/2 - (I513*sin(q2 + q3 + q4 - q5))/2 - (I623*cos(q2 + q3 + q4 + q5 + q6))/4 - (I613*sin(q2 + q3 + q4 + q5 + q6))/4 - I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4) - (I611*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (I611*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (I622*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (I622*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (I623*cos(q2 + q3 + q4 + q5 - q6))/4 - (I623*cos(q2 + q3 + q4 - q5 + q6))/4 - (I623*cos(q2 + q3 + q4 - 2*q5 + q6))/4 - (I623*cos(q2 + q3 + q4 + 2*q5 + q6))/4 - (L14^2*m6*cos(q2 + q3 + q4 - 2*q5))/4 + (L14^2*m6*cos(q2 + q3 + q4 + 2*q5))/4 - L2*L5*m5*cos(q2 + q3) - L2*L5*m6*cos(q2 + q3) - L5*L12*m4*cos(q2 + q3) - L9*L10*m3*cos(q2 + q3) - (L3*L14*m6*cos(q2 + q5))/2 - L2*L3*m5*cos(q2) - L2*L3*m6*cos(q2) - L3*L9*m3*cos(q2) - L7*L8*m2*cos(q2) - L3*L12*m4*cos(q2) - (L2*L14*m6*cos(q2 + q3 + q4 - q5))/2 - (L11*L14*m6*cos(q2 + q3 + q4 - q5))/2 - (L3*L14*m6*cos(q2 - q5))/2 - L2*L11*m6*cos(q2 + q3 + q4) - L2*L13*m5*cos(q2 + q3 + q4) - (L5*L14*m6*cos(q2 + q3 + q5))/2 - (L5*L14*m6*cos(q2 + q3 - q5))/2 + (L2*L14*m6*cos(q2 + q3 + q4 + q5))/2 - (L11*L14*m6*cos(q2 + q3 + q4 + q5))/2;
M_dynModel(1,3) = (I613*sin(q2 + q3 + q4 + q5 - q6))/4 - (I612*sin(q2 + q3 + q4 + q5 + 2*q6))/4 - (I612*sin(q2 + q3 + q4 + q5 - 2*q6))/4 - (I613*sin(q2 + q3 + q4 - q5 + q6))/4 - (I613*sin(q2 + q3 + q4 - 2*q5 + q6))/4 - (I613*sin(q2 + q3 + q4 + 2*q5 + q6))/4 + (I523*cos(q2 + q3 + q4 + q5))/2 + (I513*sin(q2 + q3 + q4 + q5))/2 - I313*cos(q2 + q3) + I323*sin(q2 + q3) + (I611*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (I611*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I611*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 - (I611*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 + (I611*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 + (I611*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 - (I622*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (I622*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I623*cos(q2 + q3 + q4 - q5 - q6))/4 + (I622*cos(q2 + q3 + q4 - 2*q5 - 2*q6))/16 + (I622*cos(q2 + q3 + q4 - 2*q5 + 2*q6))/16 - (I622*cos(q2 + q3 + q4 + 2*q5 - 2*q6))/16 - (I622*cos(q2 + q3 + q4 + 2*q5 + 2*q6))/16 + (I623*cos(q2 + q3 + q4 - 2*q5 - q6))/4 + (I623*cos(q2 + q3 + q4 + 2*q5 - q6))/4 + (I612*sin(q2 + q3 + q4 - q5 - 2*q6))/4 + (I612*sin(q2 + q3 + q4 - q5 + 2*q6))/4 + (I613*sin(q2 + q3 + q4 - q5 - q6))/4 - (I612*sin(q2 + q3 + q4 - 2*q5 - 2*q6))/8 + (I612*sin(q2 + q3 + q4 - 2*q5 + 2*q6))/8 + (I612*sin(q2 + q3 + q4 + 2*q5 - 2*q6))/8 - (I612*sin(q2 + q3 + q4 + 2*q5 + 2*q6))/8 - (I613*sin(q2 + q3 + q4 - 2*q5 - q6))/4 - (I613*sin(q2 + q3 + q4 + 2*q5 - q6))/4 - (I511*cos(q2 + q3 + q4 - 2*q5))/4 + (I511*cos(q2 + q3 + q4 + 2*q5))/4 + (I522*cos(q2 + q3 + q4 - 2*q5))/4 - (I522*cos(q2 + q3 + q4 + 2*q5))/4 + (I523*cos(q2 + q3 + q4 - q5))/2 - (I611*cos(q2 + q3 + q4 - 2*q5))/8 + (I611*cos(q2 + q3 + q4 + 2*q5))/8 - (I622*cos(q2 + q3 + q4 - 2*q5))/8 + (I622*cos(q2 + q3 + q4 + 2*q5))/8 + (I633*cos(q2 + q3 + q4 - 2*q5))/4 - (I633*cos(q2 + q3 + q4 + 2*q5))/4 - (I512*sin(q2 + q3 + q4 - 2*q5))/2 - (I512*sin(q2 + q3 + q4 + 2*q5))/2 - (I513*sin(q2 + q3 + q4 - q5))/2 - (I623*cos(q2 + q3 + q4 + q5 + q6))/4 - (I613*sin(q2 + q3 + q4 + q5 + q6))/4 - I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4) - (I611*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (I611*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (I622*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (I622*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (I623*cos(q2 + q3 + q4 + q5 - q6))/4 - (I623*cos(q2 + q3 + q4 - q5 + q6))/4 - (I623*cos(q2 + q3 + q4 - 2*q5 + q6))/4 - (I623*cos(q2 + q3 + q4 + 2*q5 + q6))/4 - (L14^2*m6*cos(q2 + q3 + q4 - 2*q5))/4 + (L14^2*m6*cos(q2 + q3 + q4 + 2*q5))/4 - L2*L5*m5*cos(q2 + q3) - L2*L5*m6*cos(q2 + q3) - L5*L12*m4*cos(q2 + q3) - L9*L10*m3*cos(q2 + q3) - (L2*L14*m6*cos(q2 + q3 + q4 - q5))/2 - (L11*L14*m6*cos(q2 + q3 + q4 - q5))/2 - L2*L11*m6*cos(q2 + q3 + q4) - L2*L13*m5*cos(q2 + q3 + q4) - (L5*L14*m6*cos(q2 + q3 + q5))/2 - (L5*L14*m6*cos(q2 + q3 - q5))/2 + (L2*L14*m6*cos(q2 + q3 + q4 + q5))/2 - (L11*L14*m6*cos(q2 + q3 + q4 + q5))/2;
M_dynModel(1,4) = (I523*cos(q2 + q3 + q4 + q5))/2 + (I513*sin(q2 + q3 + q4 + q5))/2 - (I511*cos(q2 + q3 + q4 - 2*q5))/4 + (I511*cos(q2 + q3 + q4 + 2*q5))/4 + (I522*cos(q2 + q3 + q4 - 2*q5))/4 - (I522*cos(q2 + q3 + q4 + 2*q5))/4 + (I523*cos(q2 + q3 + q4 - q5))/2 - (I512*sin(q2 + q3 + q4 - 2*q5))/2 - (I512*sin(q2 + q3 + q4 + 2*q5))/2 - (I513*sin(q2 + q3 + q4 - q5))/2 - I423*cos(q2 + q3 + q4) - I413*sin(q2 + q3 + q4) - L2*L13*m5*cos(q2 + q3 + q4) - I612*cos(q2 + q3 + q4)*cos(q1)^2*cos(q6)^2*sin(q5) - I613*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)^2*cos(q6) + I612*cos(q2 + q3 + q4)*cos(q1)^2*sin(q5)*sin(q6)^2 - I612*cos(q2 + q3 + q4)*cos(q6)^2*sin(q1)^2*sin(q5) + I613*sin(q2 + q3 + q4)*cos(q1)^2*cos(q6)*sin(q5)^2 - I613*sin(q2 + q3 + q4)*cos(q5)^2*cos(q6)*sin(q1)^2 + I623*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)^2*sin(q6) + I612*cos(q2 + q3 + q4)*sin(q1)^2*sin(q5)*sin(q6)^2 + I613*sin(q2 + q3 + q4)*cos(q6)*sin(q1)^2*sin(q5)^2 - I623*sin(q2 + q3 + q4)*cos(q1)^2*sin(q5)^2*sin(q6) + I623*sin(q2 + q3 + q4)*cos(q5)^2*sin(q1)^2*sin(q6) - I623*sin(q2 + q3 + q4)*sin(q1)^2*sin(q5)^2*sin(q6) - I623*cos(q2 + q3 + q4)*cos(q1)^2*cos(q5)*cos(q6) - I613*cos(q2 + q3 + q4)*cos(q1)^2*cos(q5)*sin(q6) - I623*cos(q2 + q3 + q4)*cos(q5)*cos(q6)*sin(q1)^2 - L2*L11*m6*cos(q2 + q3 + q4)*cos(q1)^2 - I613*cos(q2 + q3 + q4)*cos(q5)*sin(q1)^2*sin(q6) + I633*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)*sin(q5) - L2*L11*m6*cos(q2 + q3 + q4)*sin(q1)^2 + I633*sin(q2 + q3 + q4)*cos(q5)*sin(q1)^2*sin(q5) - I622*sin(q2 + q3 + q4)*cos(q5)*sin(q1)^2*sin(q5)*sin(q6)^2 - L11*L14*m6*cos(q2 + q3 + q4)*cos(q1)^2*cos(q5) - L14^2*m6*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)*sin(q5) - I611*cos(q2 + q3 + q4)*cos(q1)^2*cos(q6)*sin(q5)*sin(q6) + I622*cos(q2 + q3 + q4)*cos(q1)^2*cos(q6)*sin(q5)*sin(q6) - L11*L14*m6*cos(q2 + q3 + q4)*cos(q5)*sin(q1)^2 - L14^2*m6*sin(q2 + q3 + q4)*cos(q5)*sin(q1)^2*sin(q5) - I611*cos(q2 + q3 + q4)*cos(q6)*sin(q1)^2*sin(q5)*sin(q6) + I622*cos(q2 + q3 + q4)*cos(q6)*sin(q1)^2*sin(q5)*sin(q6) - L2*L14*m6*sin(q2 + q3 + q4)*cos(q1)^2*sin(q5) - L2*L14*m6*sin(q2 + q3 + q4)*sin(q1)^2*sin(q5) - I611*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)*cos(q6)^2*sin(q5) - I611*sin(q2 + q3 + q4)*cos(q5)*cos(q6)^2*sin(q1)^2*sin(q5) - I622*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)*sin(q5)*sin(q6)^2 + 2*I612*sin(q2 + q3 + q4)*cos(q1)^2*cos(q5)*cos(q6)*sin(q5)*sin(q6) + 2*I612*sin(q2 + q3 + q4)*cos(q5)*cos(q6)*sin(q1)^2*sin(q5)*sin(q6);
M_dynModel(1,5) = (I612*sin(q2 + q3 + q4 + q5 - 2*q6))/4 + (I612*sin(q2 + q3 + q4 + q5 + 2*q6))/4 - (I613*sin(q2 + q3 + q4 + q5 - q6))/4 - (I613*sin(q2 + q3 + q4 - q5 + q6))/4 - (I523*cos(q2 + q3 + q4 + q5))/2 - (I513*sin(q2 + q3 + q4 + q5))/2 + (I611*cos(q2 + q3 + q4 - q5 - 2*q6))/8 - (I611*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I622*cos(q2 + q3 + q4 - q5 - 2*q6))/8 + (I622*cos(q2 + q3 + q4 - q5 + 2*q6))/8 - (I623*cos(q2 + q3 + q4 - q5 - q6))/4 + (I612*sin(q2 + q3 + q4 - q5 - 2*q6))/4 + (I612*sin(q2 + q3 + q4 - q5 + 2*q6))/4 + (I613*sin(q2 + q3 + q4 - q5 - q6))/4 + (I523*cos(q2 + q3 + q4 - q5))/2 - (I611*cos(q2 + q3 + q4 - 2*q6))/4 - (I611*cos(q2 + q3 + q4 + 2*q6))/4 + (I622*cos(q2 + q3 + q4 - 2*q6))/4 + (I622*cos(q2 + q3 + q4 + 2*q6))/4 - (I513*sin(q2 + q3 + q4 - q5))/2 - (I612*sin(q2 + q3 + q4 - 2*q6))/2 + (I612*sin(q2 + q3 + q4 + 2*q6))/2 + (I623*cos(q2 + q3 + q4 + q5 + q6))/4 + (I613*sin(q2 + q3 + q4 + q5 + q6))/4 + I533*cos(q2 + q3 + q4) + (I611*cos(q2 + q3 + q4))/2 + (I622*cos(q2 + q3 + q4))/2 + (I611*cos(q2 + q3 + q4 + q5 - 2*q6))/8 - (I611*cos(q2 + q3 + q4 + q5 + 2*q6))/8 - (I622*cos(q2 + q3 + q4 + q5 - 2*q6))/8 + (I622*cos(q2 + q3 + q4 + q5 + 2*q6))/8 + (I623*cos(q2 + q3 + q4 + q5 - q6))/4 - (I623*cos(q2 + q3 + q4 - q5 + q6))/4 + L14^2*m6*cos(q2 + q3 + q4) + (L3*L14*m6*cos(q2 + q5))/2 + (L2*L14*m6*cos(q2 + q3 + q4 - q5))/2 - (L11*L14*m6*cos(q2 + q3 + q4 - q5))/2 - (L3*L14*m6*cos(q2 - q5))/2 + (L5*L14*m6*cos(q2 + q3 + q5))/2 - (L5*L14*m6*cos(q2 + q3 - q5))/2 + (L2*L14*m6*cos(q2 + q3 + q4 + q5))/2 + (L11*L14*m6*cos(q2 + q3 + q4 + q5))/2;
M_dynModel(1,6) = sin(q2 + q3 + q4)*sin(q5)*((cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6))*(I611*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I612*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I613*sin(q2 + q3 + q4)*sin(q5)) + (cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6))*(I612*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I622*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I623*sin(q2 + q3 + q4)*sin(q5)) - sin(q2 + q3 + q4)*sin(q5)*(I613*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I623*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I633*sin(q2 + q3 + q4)*sin(q5))) - (cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5))*((cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6))*(I611*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I612*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I613*sin(q2 + q3 + q4)*sin(q5)) - (sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) + sin(q2 + q3 + q4)*cos(q6)*sin(q1))*(I612*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I622*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I623*sin(q2 + q3 + q4)*sin(q5)) + (cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5))*(I613*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I623*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I633*sin(q2 + q3 + q4)*sin(q5))) - (cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5))*((cos(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*sin(q6))*(I611*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I612*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I613*sin(q2 + q3 + q4)*sin(q5)) - (sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*cos(q6))*(I612*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I622*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I623*sin(q2 + q3 + q4)*sin(q5)) + (cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5))*(I613*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + I623*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - I633*sin(q2 + q3 + q4)*sin(q5)));
M_dynModel(2,1) = M_dynModel(1,2);
M_dynModel(2,2) = I233 + I333 + I433 + I511 + I622 + L3^2*m3 + L3^2*m4 + L3^2*m5 + L3^2*m6 + L5^2*m4 + L5^2*m5 + L8^2*m2 + L5^2*m6 + L10^2*m3 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) + 2*L3*L5*m4*cos(q3) + 2*L3*L5*m5*cos(q3) + 2*L3*L5*m6*cos(q3) + 2*L3*L10*m3*cos(q3) + 2*L5*L11*m6*cos(q4) + 2*L5*L13*m5*cos(q4) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6) + 2*L3*L11*m6*cos(q3)*cos(q4) + 2*L3*L13*m5*cos(q3)*cos(q4) - 2*L3*L11*m6*sin(q3)*sin(q4) - 2*L3*L13*m5*sin(q3)*sin(q4) + 2*L5*L14*m6*sin(q4)*sin(q5) + 2*L3*L14*m6*cos(q3)*sin(q4)*sin(q5) + 2*L3*L14*m6*cos(q4)*sin(q3)*sin(q5);
M_dynModel(2,3) = I333 + I433 + I511 + I622 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L10^2*m3 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) + L3*L5*m4*cos(q3) + L3*L5*m5*cos(q3) + L3*L5*m6*cos(q3) + L3*L10*m3*cos(q3) + 2*L5*L11*m6*cos(q4) + 2*L5*L13*m5*cos(q4) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6) + L3*L11*m6*cos(q3)*cos(q4) + L3*L13*m5*cos(q3)*cos(q4) - L3*L11*m6*sin(q3)*sin(q4) - L3*L13*m5*sin(q3)*sin(q4) + 2*L5*L14*m6*sin(q4)*sin(q5) + L3*L14*m6*cos(q3)*sin(q4)*sin(q5) + L3*L14*m6*cos(q4)*sin(q3)*sin(q5);
M_dynModel(2,4) = I433 + I511 + I622 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) + L5*L11*m6*cos(q4) + L5*L13*m5*cos(q4) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6) + L3*L11*m6*cos(q3)*cos(q4) + L3*L13*m5*cos(q3)*cos(q4) - L3*L11*m6*sin(q3)*sin(q4) - L3*L13*m5*sin(q3)*sin(q4) + L5*L14*m6*sin(q4)*sin(q5) + L3*L14*m6*cos(q3)*sin(q4)*sin(q5) + L3*L14*m6*cos(q4)*sin(q3)*sin(q5);
M_dynModel(2,5) = I523*cos(q5) + I513*sin(q5) + I612*sin(q5) - I623*cos(q5)*cos(q6) - I613*cos(q5)*sin(q6) - 2*I612*cos(q6)^2*sin(q5) - L11*L14*m6*cos(q5) - I611*cos(q6)*sin(q5)*sin(q6) + I622*cos(q6)*sin(q5)*sin(q6) - L5*L14*m6*cos(q4)*cos(q5) + L3*L14*m6*cos(q5)*sin(q3)*sin(q4) - L3*L14*m6*cos(q3)*cos(q4)*cos(q5);
M_dynModel(2,6) = I633*cos(q5) + I613*cos(q6)*sin(q5) - I623*sin(q5)*sin(q6);
M_dynModel(3,1) = M_dynModel(1,3);
M_dynModel(3,2) = M_dynModel(2,3);
M_dynModel(3,3) = I333 + I433 + I511 + I622 + L5^2*m4 + L5^2*m5 + L5^2*m6 + L10^2*m3 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) + 2*L5*L11*m6*cos(q4) + 2*L5*L13*m5*cos(q4) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6) + 2*L5*L14*m6*sin(q4)*sin(q5);
M_dynModel(3,4) = I433 + I511 + I622 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) + L5*L11*m6*cos(q4) + L5*L13*m5*cos(q4) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6) + L5*L14*m6*sin(q4)*sin(q5);
M_dynModel(3,5) = I523*cos(q5) + I513*sin(q5) + I612*sin(q5) - I623*cos(q5)*cos(q6) - I613*cos(q5)*sin(q6) - 2*I612*cos(q6)^2*sin(q5) - L11*L14*m6*cos(q5) - I611*cos(q6)*sin(q5)*sin(q6) + I622*cos(q6)*sin(q5)*sin(q6) - L5*L14*m6*cos(q4)*cos(q5);
M_dynModel(3,6) = I633*cos(q5) + I613*cos(q6)*sin(q5) - I623*sin(q5)*sin(q6);
M_dynModel(4,1) = M_dynModel(1,4);
M_dynModel(4,2) = M_dynModel(2,4);
M_dynModel(4,3) = M_dynModel(3,4);
M_dynModel(4,4) = I433 + I511 + I622 + L11^2*m6 + L13^2*m5 + L14^2*m6 - I511*cos(q5)^2 + I522*cos(q5)^2 + I611*cos(q6)^2 - I622*cos(q5)^2 - I622*cos(q6)^2 + I633*cos(q5)^2 + I512*sin(2*q5) - I612*sin(2*q6) - I611*cos(q5)^2*cos(q6)^2 + I622*cos(q5)^2*cos(q6)^2 - L14^2*m6*cos(q5)^2 + 2*I613*cos(q5)*cos(q6)*sin(q5) - 2*I623*cos(q5)*sin(q5)*sin(q6) + 2*I612*cos(q5)^2*cos(q6)*sin(q6);
M_dynModel(4,5) = I523*cos(q5) + I513*sin(q5) + I612*sin(q5) - I623*cos(q5)*cos(q6) - I613*cos(q5)*sin(q6) - 2*I612*cos(q6)^2*sin(q5) - L11*L14*m6*cos(q5) - I611*cos(q6)*sin(q5)*sin(q6) + I622*cos(q6)*sin(q5)*sin(q6);
M_dynModel(4,6) = I633*cos(q5) + I613*cos(q6)*sin(q5) - I623*sin(q5)*sin(q6);
M_dynModel(5,1) = M_dynModel(1,5);
M_dynModel(5,2) = M_dynModel(2,5);
M_dynModel(5,3) = M_dynModel(3,5);
M_dynModel(5,4) = M_dynModel(4,5);
M_dynModel(5,5) = m6*L14^2 + I533 + I611/2 + I622/2 - (I611*cos(2*q6))/2 + (I622*cos(2*q6))/2 + I612*sin(2*q6);
M_dynModel(5,6) = sin(q2 + q3 + q4)*sin(q5)*((I612*cos(q6) + I611*sin(q6))*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)) + (I622*cos(q6) + I612*sin(q6))*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)) - sin(q2 + q3 + q4)*sin(q5)*(I623*cos(q6) + I613*sin(q6))) - (cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5))*((I612*cos(q6) + I611*sin(q6))*(cos(q1)*cos(q6)*sin(q5) - sin(q2 + q3 + q4)*sin(q1)*sin(q6) + cos(q2 + q3 + q4)*cos(q5)*cos(q6)*sin(q1)) + (I623*cos(q6) + I613*sin(q6))*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) - (I622*cos(q6) + I612*sin(q6))*(sin(q2 + q3 + q4)*cos(q6)*sin(q1) + cos(q1)*sin(q5)*sin(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)*sin(q6))) - (cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5))*((I612*cos(q6) + I611*sin(q6))*(sin(q2 + q3 + q4)*cos(q1)*sin(q6) + cos(q6)*sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)*cos(q6)) + (I622*cos(q6) + I612*sin(q6))*(sin(q2 + q3 + q4)*cos(q1)*cos(q6) - sin(q1)*sin(q5)*sin(q6) + cos(q2 + q3 + q4)*cos(q1)*cos(q5)*sin(q6)) + (I623*cos(q6) + I613*sin(q6))*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)));
M_dynModel(6,1) = M_dynModel(1,6);
M_dynModel(6,2) = M_dynModel(2,6);
M_dynModel(6,3) = M_dynModel(3,6);
M_dynModel(6,4) = M_dynModel(4,6);
M_dynModel(6,5) = M_dynModel(5,6);
M_dynModel(6,6) = I633;

test_M_2 = simplify(M-M_dynModel);

% coriolis & centripetal effects matrix
C_dynModel(1,1) = 0;
C_dynModel(1,2) = 0;
C_dynModel(1,3) = 0;
C_dynModel(1,4) = 0;
C_dynModel(1,5) = 0;
C_dynModel(1,6) = 0;
C_dynModel(2,1) = 0;
C_dynModel(2,2) = 0;
C_dynModel(2,3) = 0;
C_dynModel(2,4) = 0;
C_dynModel(2,5) = 0;
C_dynModel(2,6) = 0;
C_dynModel(3,1) = 0;
C_dynModel(3,2) = 0;
C_dynModel(3,3) = 0;
C_dynModel(3,4) = 0;
C_dynModel(3,5) = 0;
C_dynModel(3,6) = 0;
C_dynModel(4,1) = 0;
C_dynModel(4,2) = 0;
C_dynModel(4,3) = 0;
C_dynModel(4,4) = 0;
C_dynModel(4,5) = 0;
C_dynModel(4,6) = 0;
C_dynModel(5,1) = 0;
C_dynModel(5,2) = 0;
C_dynModel(5,3) = 0;
C_dynModel(5,4) = 0;
C_dynModel(5,5) = 0;
C_dynModel(5,6) = 0;
C_dynModel(6,1) = 0;
C_dynModel(6,2) = 0;
C_dynModel(6,3) = 0;
C_dynModel(6,4) = 0;
C_dynModel(6,5) = 0;
C_dynModel(6,6) = 0;
test_C = simplify(C-C_dynModel);

% gravitational torques vector
G_dynModel(1,1) = g*gy*m6*(L14*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)) ...
                  - cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L2*sin(q1) - L11*sin(q2 + q3 + q4)*cos(q1)) ...
                  + g*gx*m4*(L12*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2))) ...
                  - g*gy*m4*(cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) - L12*sin(q1)) ...
                  + g*gx*m6*(L14*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)) ...
                  + L2*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) + L11*sin(q2 + q3 + q4)*sin(q1)) ...
                  - g*gy*m3*(L3*cos(q1)*sin(q2) - L9*sin(q1) + L10*cos(q1)*cos(q2)*sin(q3) ...
                  + L10*cos(q1)*cos(q3)*sin(q2)) + g*gx*m3*(L9*cos(q1) + L3*sin(q1)*sin(q2) ...
                  + L10*cos(q2)*sin(q1)*sin(q3) + L10*cos(q3)*sin(q1)*sin(q2)) ...
                  - g*gy*m5*(cos(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) - L2*sin(q1) ...
                  + L13*sin(q2 + q3 + q4)*cos(q1)) + g*gx*m5*(L2*cos(q1) + sin(q1)*(L5*sin(q2 + q3) + L3*sin(q2)) ...
                  + L13*sin(q2 + q3 + q4)*sin(q1)) + g*gx*m2*(L7*cos(q1) + L8*sin(q1)*sin(q2)) ...
                  + g*gy*m2*(L7*sin(q1) - L8*cos(q1)*sin(q2));
G_dynModel(2,1) = - g*gz*m6*(L5*sin(q2 + q3) + L3*sin(q2) + L11*sin(q2 + q3 + q4) ...
                  - L14*cos(q2 + q3 + q4)*sin(q5)) - g*gz*m5*(L5*sin(q2 + q3) ...
                  + L3*sin(q2) + L13*sin(q2 + q3 + q4)) - g*gz*m4*(L5*sin(q2 + q3) ...
                  + L3*sin(q2)) - g*gz*m3*(L10*sin(q2 + q3) + L3*sin(q2)) ...
                  - g*gx*m6*cos(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L11*cos(q2 + q3 + q4) ...
                  + L14*sin(q2 + q3 + q4)*sin(q5)) - g*gy*m6*sin(q1)*(L5*cos(q2 + q3) ...
                  + L3*cos(q2) + L11*cos(q2 + q3 + q4) + L14*sin(q2 + q3 + q4)*sin(q5)) ...
                  - g*gx*m5*cos(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L13*cos(q2 + q3 + q4)) ...
                  - g*gy*m5*sin(q1)*(L5*cos(q2 + q3) + L3*cos(q2) + L13*cos(q2 + q3 + q4)) ...
                  - g*gx*m4*cos(q1)*(L5*cos(q2 + q3) + L3*cos(q2)) - g*gx*m3*cos(q1)*(L10*cos(q2 + q3) ...
                  + L3*cos(q2)) - g*gy*m4*sin(q1)*(L5*cos(q2 + q3) + L3*cos(q2)) ...
                  - g*gy*m3*sin(q1)*(L10*cos(q2 + q3) + L3*cos(q2)) - L8*g*gz*m2*sin(q2) ...
                  - L8*g*gx*m2*cos(q1)*cos(q2) - L8*g*gy*m2*cos(q2)*sin(q1);
G_dynModel(3,1) = - g*gz*m5*(L5*sin(q2 + q3) + L13*sin(q2 + q3 + q4)) - g*gz*m6*(L5*sin(q2 + q3) ...
                  + L11*sin(q2 + q3 + q4) - L14*cos(q2 + q3 + q4)*sin(q5)) ...
                  - g*gx*m6*cos(q1)*(L5*cos(q2 + q3) + L11*cos(q2 + q3 + q4) ...
                  + L14*sin(q2 + q3 + q4)*sin(q5)) - L5*g*gz*m4*sin(q2 + q3) ...
                  - L10*g*gz*m3*sin(q2 + q3) - g*gy*m6*sin(q1)*(L5*cos(q2 + q3) ...
                  + L11*cos(q2 + q3 + q4) + L14*sin(q2 + q3 + q4)*sin(q5)) ...
                  - g*gx*m5*cos(q1)*(L5*cos(q2 + q3) + L13*cos(q2 + q3 + q4)) ...
                  - g*gy*m5*sin(q1)*(L5*cos(q2 + q3) + L13*cos(q2 + q3 + q4)) ...
                  - L5*g*gx*m4*cos(q2 + q3)*cos(q1) - L10*g*gx*m3*cos(q2 + q3)*cos(q1) ...
                  - L5*g*gy*m4*cos(q2 + q3)*sin(q1) - L10*g*gy*m3*cos(q2 + q3)*sin(q1);
G_dynModel(4,1) = - g*gz*m6*(L11*sin(q2 + q3 + q4) - L14*cos(q2 + q3 + q4)*sin(q5)) ...
                  - L13*g*gz*m5*sin(q2 + q3 + q4) - g*gx*m6*cos(q1)*(L11*cos(q2 + q3 + q4) ...
                  + L14*sin(q2 + q3 + q4)*sin(q5)) - g*gy*m6*sin(q1)*(L11*cos(q2 + q3 + q4) ...
                  + L14*sin(q2 + q3 + q4)*sin(q5)) - L13*g*gx*m5*cos(q2 + q3 + q4)*cos(q1) ...
                  - L13*g*gy*m5*cos(q2 + q3 + q4)*sin(q1);
G_dynModel(5,1) = L14*g*gy*m6*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) ...
                  - L14*g*gx*m6*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) ...
                  + L14*g*gz*m6*sin(q2 + q3 + q4)*cos(q5);
G_dynModel(6,1) = 0;
test_G = simplify(G-G_dynModel);

% print result
fprintf('--------------------------\n');
fprintf('The results of matrix tests computed symbolically & used in dynamic model:\n');
if(isAlways(test_M_2 == zeros(3)))
   fprintf(' - Inertia matrix M: Correct matrix used.\n');
else
   fprintf(' - Inertia matrix M: INCORRECT matrix used.\n');
end
if(isAlways(test_C == zeros(3)))
   fprintf(' - Coriolis & centripetal effects matrix C: Correct matrix used.\n');
else
   fprintf(' - Coriolis & centripetal effects matrix C: INCORRECT matrix used.\n');
end
if(isAlways(test_G == zeros(3,1)))
   fprintf(' - Gravitational torques matrix M: Correct matrix used.\n');
else
   fprintf(' - Gravitational torques matrix M: INCORRECT matrix used.\n');
end
fprintf('--------------------------\n');

%% dynamics equation
dyn_eq = simplify(M*[ppq1r;ppq2r;ppq3r]+C*[pq1r;pq2r;pq3r]+G);
 
%% robot regressor

% % Define the vector of parameters (symbolic form)
% Theta(1,1)=I133;
% Theta(2,1)=I211;
% Theta(3,1)=I222;
% Theta(4,1)=I311;
% Theta(5,1)=I312;
% Theta(6,1)=L3^2*m3;
% Theta(7,1)=L7^2*m2;
% Theta(8,1)=L8^2*m2;
% Theta(9,1)=L9^2*m3;
% Theta(10,1)=L10^2*m3;
% Theta(11,1)=I212;
% Theta(12,1)=L3*L10*m3;
% Theta(13,1)=I313;
% Theta(14,1)=I323;
% Theta(15,1)=I213;
% Theta(16,1)=I223;
% Theta(17,1)=L3*L9*m3;
% Theta(18,1)=L7*L8*m2;
% Theta(19,1)=L9*L10*m3;
% Theta(20,1)=I233;
% Theta(21,1)=I333;
% Theta(22,1)=I322;
% Theta(23,1)=g*gx*m3*L9;
% Theta(24,1)=g*gx*m3*L3;
% Theta(25,1)=g*gx*m3*L10;
% Theta(26,1)=g*gy*m3*L3;
% Theta(27,1)=g*gy*m3*L9;
% Theta(28,1)=g*gy*m3*L10;
% Theta(29,1)=g*gx*m2*L7;
% Theta(30,1)=g*gx*m2*L8;
% Theta(31,1)=g*gy*m2*L7;
% Theta(32,1)=g*gy*m2*L8;
% Theta(33,1)=g*gz*m3*L10;
% Theta(34,1)=g*gz*m3*L3;
% Theta(35,1)=g*gz*m2*L8;
% 
% % Define the matrix of states (symbolic form)
% Yr(1:3,1)=[ppq1r;0;0];
% Yr(1:3,2)=[(1+cos(2*q2))/2*ppq1r - sin(2*q2)/2*(pq2*pq1r+pq1*pq2r);
%            sin(2*q2)/2*pq1*pq1r;
%            0];
% Yr(1:3,3)=[(1-cos(2*q2))/2*ppq1r + sin(2*q2)/2*(pq2*pq1r+pq1*pq2r);% + sin(2*q2+2*q3)/2*(pq2+pq3)*pq1r;       % It's weird that the indices do not rotate!
%            -sin(2*q2)/2*pq1*pq1r;
%            0];
% Yr(1:3,4)=[(1+cos(2*q2+2*q3))/2*ppq1r - sin(2*q2+2*q3)/2*(pq1*pq2r+pq2*pq1r+pq1*pq3r+pq3*pq1r);
%            sin(2*q2+2*q3)/2*pq1*pq1r;
%            sin(2*q2+2*q3)/2*pq1*pq1r];
% Yr(1:3,5)=[-sin(2*q2+2*q3)*ppq1r - cos(2*q2+2*q3)*((pq2+pq3)*pq1r+pq1*(pq2r+pq3r));
%            cos(2*q2+2*q3)*pq1*pq1r;
%            cos(2*q2+2*q3)*pq1*pq1r];
% Yr(1:3,6)=[(1-cos(2*q2))/2*ppq1r + sin(2*q2)/2*(pq2*pq1r+pq1*pq2r);
%            ppq2r - sin(2*q2)/2*pq1*pq1r;
%            0];       
% Yr(1:3,7)=[ppq1r;0;0]; 
% Yr(1:3,8)=[(1-cos(2*q2))/2*ppq1r + sin(2*q2)/2*(pq2*pq1r+pq1*pq2r);
%            ppq2r - sin(2*q2)/2*pq1*pq1r;
%            0];
% Yr(1:3,9)=[ppq1r;0;0]; 
% Yr(1:3,10)=[(1-cos(2*q2+2*q3))/2*ppq1r + sin(2*q2+2*q3)/2*((pq2+pq3)*pq1r+pq1*(pq2r+pq3r));
%             ppq2r + ppq3r - sin(2*q2+2*q3)/2*pq1*pq1r;
%             ppq2r + ppq3r - sin(2*q2+2*q3)/2*pq1*pq1r]; 
% Yr(1:3,11)=[-sin(2*q2)*ppq1r - cos(2*q2)*(pq2*pq1r+pq1*pq2r);
%             cos(2*q2)*pq1*pq1r;
%             0]; 
% Yr(1:3,12)=[(cos(q3)-cos(2*q2+q3))*ppq1r - sin(q3)/2*(pq3*pq1r+pq1*pq3r) ...
%               + sin(2*q2+q3)/2*(2*pq1*pq2r+2*pq2*pq1r+pq1*pq3r+pq3*pq1r);
%             cos(q3)*(2*ppq2r+ppq3r) - sin(2*q2+q3)*pq1*pq1r - sin(q3)*(pq3*pq2r+(pq2+pq3)*pq3r);
%             cos(q3)*ppq2r + (sin(q3)-sin(2*q2+q3))/2*pq1*pq1r + sin(q3)*pq2*pq2r]; 
% Yr(1:3,13)=[cos(q2+q3)*(ppq2r+ppq3r) - sin(q2+q3)*(pq2+pq3)*(pq2r+pq3r);
%             cos(q2+q3)*ppq1r;
%             cos(q2+q3)*ppq1r];
% Yr(1:3,14)=[-sin(q2+q3)*(ppq2r+ppq3r) - cos(q2+q3)*(pq2+pq3)*(pq2r+pq3r);
%             -sin(q2+q3)*ppq1r;
%             -sin(q2+q3)*ppq1r];
% Yr(1:3,15)=[cos(q2)*ppq2r - sin(q2)*pq2*pq2r;%- sin(2*q2)*pq2*pq2r;
%             cos(q2)*ppq1r;
%             0];
% Yr(1:3,16)=[-sin(q2)*ppq2r - cos(q2)*pq2*pq2r;
%             -sin(q2)*ppq1r;
%             0];
% Yr(1:3,17)=[-cos(q2)*ppq2r + sin(q2)*pq2*pq2r;
%             -cos(q2)*ppq1r;
%             0];
% Yr(1:3,18)=[-cos(q2)*ppq2r + sin(q2)*pq2*pq2r;
%             -cos(q2)*ppq1r;
%             0];
% Yr(1:3,19)=[-cos(q2+q3)*(ppq2r+ppq3r) + sin(q2+q3)*(pq2+pq3)*(pq2r+pq3r);
%             -cos(q2+q3)*ppq1r;
%             -cos(q2+q3)*ppq1r];
% Yr(1:3,20)=[0;ppq2r;0]; 
% Yr(1:3,21)=[0;ppq2r+ppq3r;ppq2r+ppq3r]; 
% Yr(1:3,22)=[(1-cos(2*q2+2*q3))/2*ppq1r + sin(2*q2+2*q3)/2*(pq1*pq2r+pq2*pq1r+pq1*pq3r+pq3*pq1r);
%             -sin(2*q2+2*q3)/2*pq1*pq1r;
%             -sin(2*q2+2*q3)/2*pq1*pq1r];
% Yr(1:3,23)=[cos(q1);0;0]; 
% Yr(1:3,24)=[sin(q1)*sin(q2);-cos(q1)*cos(q2);0]; 
% Yr(1:3,25)=[sin(q1)*(cos(q2)*sin(q3)+sin(q2)*cos(q3));
%             -cos(q1)*cos(q2+q3);
%             -cos(q1)*(cos(q2)*cos(q3)-sin(q2)*sin(q3))];
% Yr(1:3,26)=[-cos(q1)*sin(q2);-sin(q1)*cos(q2);0]; 
% Yr(1:3,27)=[sin(q1);0;0]; 
% Yr(1:3,28)=[-cos(q1)*(cos(q2)*sin(q3)+sin(q2)*cos(q3));
%             -sin(q1)*cos(q2+q3);
%             -sin(q1)*(cos(q2)*cos(q3)-sin(q2)*sin(q3))];
% Yr(1:3,29)=[cos(q1);0;0]; 
% Yr(1:3,30)=[sin(q1)*sin(q2);-cos(q1)*cos(q2);0]; 
% Yr(1:3,31)=[sin(q1);0;0]; 
% Yr(1:3,32)=[-cos(q1)*sin(q2);-sin(q1)*cos(q2);0]; 
% Yr(1:3,33)=[0;-sin(q2+q3);-sin(q2+q3)];
% Yr(1:3,34)=[0;-sin(q2);0]; 
% Yr(1:3,35)=[0;-sin(q2);0]; 
% 
%         
% % dynamic eq. from regressor
% dyn_eq_reg = Yr*Theta;
% 
% % test result
% test_dyn_eq = simplify( dyn_eq_reg - dyn_eq );
% 
% % print result
% fprintf('--------------------------\n');
% fprintf('The results of matrix tests for the motion equations:\n');
% if(isAlways(test_dyn_eq == zeros(3,1)))
%    fprintf(' - Dynamic regressor: Valid form.\n');
% else
%    fprintf(' - Dynamic regressor: Invalid form.\n');
% end
% fprintf('--------------------------\n');

%% File Creation 
% Relative transforms for each joint
fid = fopen('rel_T_i_UR10.m','w');
fprintf(fid,'function [T_1_0,T_2_1,T_3_2,T_4_3,T_5_4,T_6_5] = rel_T_i_UR10(u)\n\n');
fprintf(fid,'%% This file was generated automatically by "symModel_UR10".\n\n');
fprintf(fid,'%%%% parameters\n');
fprintf(fid,'%%Joint Position\n');
fprintf(fid,'q1=u(1);\n');
fprintf(fid,'q2=u(2);\n');
fprintf(fid,'q3=u(3);\n');
fprintf(fid,'q4=u(4);\n');
fprintf(fid,'q5=u(5);\n');
fprintf(fid,'q6=u(6);\n');
fprintf(fid,'Q=[q1;q2;q3;q4;q5;q6];\n\n');
fprintf(fid,'%%Kinematic Parameters\n');
fprintf(fid,'L1=u(7);\n');
fprintf(fid,'L2=u(8);\n');
fprintf(fid,'L3=u(9);\n');
fprintf(fid,'L4=u(10);\n');
fprintf(fid,'L5=u(11);\n');
fprintf(fid,'L6=u(12);\n');
fprintf(fid,'L7=u(13);\n');
fprintf(fid,'L8=u(14);\n');
fprintf(fid,'L9=u(15);\n');
fprintf(fid,'L10=u(16);\n');
fprintf(fid,'L11=u(17);\n');
fprintf(fid,'L12=u(18);\n');
fprintf(fid,'L13=u(19);\n');
fprintf(fid,'L14=u(20);\n\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_1_0(%d,%d)=%s;\n',i,j,char(relHt{1}(i,j)));
    end
end
fprintf(fid,'\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_2_1(%d,%d)=%s;\n',i,j,char(relHt{2}(i,j)));
    end
end
fprintf(fid,'\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_3_2(%d,%d)=%s;\n',i,j,char(relHt{3}(i,j)));
    end
end
fprintf(fid,'\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_4_3(%d,%d)=%s;\n',i,j,char(relHt{4}(i,j)));
    end
end
fprintf(fid,'\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_5_4(%d,%d)=%s;\n',i,j,char(relHt{5}(i,j)));
    end
end
fprintf(fid,'\n');
for i=1:4
    for j=1:4
        fprintf(fid,'T_6_5(%d,%d)=%s;\n',i,j,char(relHt{6}(i,j)));
    end
end
fprintf(fid,'\n');
fprintf(fid,'end \n');
fclose(fid);

% Relative homogeneous transforms for centers of mass
fid_2 = fopen('rel_T_cmi_UR10.m','w');
fprintf(fid_2,'function [T_cm1_0,T_cm2_1,T_cm3_2,T_cm4_3,T_cm5_4,T_cm6_5] = rel_T_cmi_UR10(u)\n\n');
fprintf(fid_2,'%% This file was generated automatically by "symModel_UR10".\n\n');
fprintf(fid_2,'%%%% parameters\n');
fprintf(fid_2,'%%Joint Position\n');
fprintf(fid_2,'q1=u(1);\n');
fprintf(fid_2,'q2=u(2);\n');
fprintf(fid_2,'q3=u(3);\n');
fprintf(fid_2,'q4=u(4);\n');
fprintf(fid_2,'q5=u(5);\n');
fprintf(fid_2,'q6=u(6);\n');
fprintf(fid_2,'Q=[q1;q2;q3;q4;q5;q6];\n\n');
fprintf(fid_2,'%%Kinematic Parameters\n');
fprintf(fid_2,'L1=u(7);\n');
fprintf(fid_2,'L2=u(8);\n');
fprintf(fid_2,'L3=u(9);\n');
fprintf(fid_2,'L4=u(10);\n');
fprintf(fid_2,'L5=u(11);\n');
fprintf(fid_2,'L6=u(12);\n');
fprintf(fid_2,'L7=u(13);\n');
fprintf(fid_2,'L8=u(14);\n');
fprintf(fid_2,'L9=u(15);\n');
fprintf(fid_2,'L10=u(16);\n');
fprintf(fid_2,'L11=u(17);\n');
fprintf(fid_2,'L12=u(18);\n');
fprintf(fid_2,'L13=u(19);\n');
fprintf(fid_2,'L14=u(20);\n\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm1_0(%d,%d)=%s;\n',i,j,char(relHt{7}(i,j)));
    end
end
fprintf(fid_2,'\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm2_1(%d,%d)=%s;\n',i,j,char(relHt{8}(i,j)));
    end
end
fprintf(fid_2,'\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm3_2(%d,%d)=%s;\n',i,j,char(relHt{9}(i,j)));
    end
end
fprintf(fid_2,'\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm4_3(%d,%d)=%s;\n',i,j,char(relHt{10}(i,j)));
    end
end
fprintf(fid_2,'\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm5_4(%d,%d)=%s;\n',i,j,char(relHt{11}(i,j)));
    end
end
fprintf(fid_2,'\n');
for i=1:4
    for j=1:4
        fprintf(fid_2,'T_cm6_5(%d,%d)=%s;\n',i,j,char(relHt{12}(i,j)));
    end
end
fprintf(fid_2,'\n');
fprintf(fid_2,'end \n');
fclose(fid_2);

% Forward kinematics for the end-effector wrt robot base
fid_3=fopen('FK_UR10.m','w');
fprintf(fid_3,'function X_ef_0 = FK_UR10(u)\n\n');
fprintf(fid_3,'%% This file was generated automatically by "symModel_UR10".\n\n');
fprintf(fid_3,'%%%% parameters\n');
fprintf(fid_3,'%%Joint Position\n');
fprintf(fid_3,'q1=u(1);\n');
fprintf(fid_3,'q2=u(2);\n');
fprintf(fid_3,'q3=u(3);\n');
fprintf(fid_3,'q4=u(4);\n');
fprintf(fid_3,'q5=u(5);\n');
fprintf(fid_3,'q6=u(6);\n');
fprintf(fid_3,'Q=[q1;q2;q3;q4;q5;q6];\n\n');
fprintf(fid_3,'%%Kinematic Parameters\n');
fprintf(fid_3,'L1=u(7);\n');
fprintf(fid_3,'L2=u(8);\n');
fprintf(fid_3,'L3=u(9);\n');
fprintf(fid_3,'L4=u(10);\n');
fprintf(fid_3,'L5=u(11);\n');
fprintf(fid_3,'L6=u(12);\n');
fprintf(fid_3,'L7=u(13);\n');
fprintf(fid_3,'L8=u(14);\n');
fprintf(fid_3,'L9=u(15);\n');
fprintf(fid_3,'L10=u(16);\n');
fprintf(fid_3,'L11=u(17);\n');
fprintf(fid_3,'L12=u(18);\n');
fprintf(fid_3,'L13=u(19);\n');
fprintf(fid_3,'L14=u(20);\n\n');
for i=1:4
    for j=1:4
        fprintf(fid_3,'T_6_0(%d,%d)=%s;\n',i,j,char(T_i_0{6}(i,j)));
    end
end
fprintf(fid_3,'\n'); 
fprintf(fid_3,'phi_6_0 = %s;\n',char(phi_EF_0));
fprintf(fid_3,'theta_6_0 = %s;\n',char(theta_EF_0));
fprintf(fid_3,'psi_6_0 = %s;\n\n',char(psi_EF_0));
fprintf(fid_3,'X_ef_0 = [T_6_0(1,4);T_6_0(2,4);T_6_0(3,4);psi_6_0;theta_6_0;phi_6_0];\n\n');
fprintf(fid_3,'end \n');
fclose(fid_3);

% Jacobian matrix of end-effector
fid_4 = fopen('diff_FK_UR10.m','w');
fprintf(fid_4,'function [XOut] = diff_FK_UR10(u)\n\n');
fprintf(fid_4,'%% This file was generated automatically by "symModel_UR10".\n\n');
fprintf(fid_4,'%%%% parameters\n');
fprintf(fid_4,'%%Joint Position\n');
fprintf(fid_4,'q1=u(1);\n');
fprintf(fid_4,'q2=u(2);\n');
fprintf(fid_4,'q3=u(3);\n');
fprintf(fid_4,'q4=u(4);\n');
fprintf(fid_4,'q5=u(5);\n');
fprintf(fid_4,'q6=u(6);\n');
fprintf(fid_4,'Q=[q1;q2;q3;q4;q5;q6];\n\n');
fprintf(fid_4,'%%Kinematic Parameters\n');
fprintf(fid_4,'L1=u(7);\n');
fprintf(fid_4,'L2=u(8);\n');
fprintf(fid_4,'L3=u(9);\n');
fprintf(fid_4,'L4=u(10);\n');
fprintf(fid_4,'L5=u(11);\n');
fprintf(fid_4,'L6=u(12);\n');
fprintf(fid_4,'L7=u(13);\n');
fprintf(fid_4,'L8=u(14);\n');
fprintf(fid_4,'L9=u(15);\n');
fprintf(fid_4,'L10=u(16);\n');
fprintf(fid_4,'L11=u(17);\n');
fprintf(fid_4,'L12=u(18);\n');
fprintf(fid_4,'L13=u(19);\n');
fprintf(fid_4,'L14=u(20);\n\n');
fprintf(fid_4,'%%Jacobian\n');
for i=1:6
    for j=1:6
        fprintf(fid_4,'J_6_0(%d,%d)=%s;\n',i,j,char(J_ef_0(i,j)));
    end
end
fprintf(fid_4,'\n');
fprintf(fid_4,'%%Derivative of jacobian\n');
for i=1:6
    for j=1:6
        fprintf(fid_4,'pJ_6_0(%d,%d)=%s;\n',i,j,char(pJ_ef_0(i,j)));
    end
end
fprintf(fid_4,'\n');
fprintf(fid_4,'%%Output\n');
fprintf(fid_4,'XOut=[J_6_0,pJ_6_0];\n');
fprintf(fid_4,'end \n');
fclose(fid_4);

% Dynamic model
fid_5 = fopen('dynamics_UR10.m','w');
fprintf(fid_5,'function [ppQ] = dynamics_UR10(u)\n\n');
fprintf(fid_5,'%% This file was generated automatically by "symModel_UR10".\n\n');
fprintf(fid_5,'%%%% parameters\n');
fprintf(fid_5,'%%Joint Position\n');
fprintf(fid_5,'q1=u(1);\n');
fprintf(fid_5,'q2=u(2);\n');
fprintf(fid_5,'q3=u(3);\n');
fprintf(fid_5,'q4=u(4);\n');
fprintf(fid_5,'q5=u(5);\n');
fprintf(fid_5,'q6=u(6);\n');
fprintf(fid_5,'Q=[q1;q2;q3;q4;q5;q6];\n\n');
fprintf(fid_5,'%%Joint Velocity\n');
fprintf(fid_5,'pq1=u(7);\n');
fprintf(fid_5,'pq2=u(8);\n');
fprintf(fid_5,'pq3=u(9);\n');
fprintf(fid_5,'pq4=u(10);\n');
fprintf(fid_5,'pq5=u(11);\n');
fprintf(fid_5,'pq6=u(12);\n');
fprintf(fid_5,'pQ=[pq1;pq2;pq3;pq4;pq5;pq6];\n\n');
fprintf(fid_5,'%%Kinematic Parameters\n');
fprintf(fid_5,'L1=u(13);\n');
fprintf(fid_5,'L2=u(14);\n');
fprintf(fid_5,'L3=u(15);\n');
fprintf(fid_5,'L4=u(16);\n');
fprintf(fid_5,'L5=u(17);\n');
fprintf(fid_5,'L6=u(18);\n');
fprintf(fid_5,'L7=u(19);\n');
fprintf(fid_5,'L8=u(20);\n');
fprintf(fid_5,'L9=u(21);\n');
fprintf(fid_5,'L10=u(22);\n');
fprintf(fid_5,'L11=u(23);\n');
fprintf(fid_5,'L12=u(24);\n');
fprintf(fid_5,'L13=u(25);\n');
fprintf(fid_5,'L14=u(26);\n\n');
fprintf(fid_5,'%%Dynamic Parameters\n');
fprintf(fid_5,'m1=u(27);\n');
fprintf(fid_5,'m2=u(28);\n');
fprintf(fid_5,'m3=u(29);\n');
fprintf(fid_5,'m4=u(30);\n');
fprintf(fid_5,'m5=u(31);\n');
fprintf(fid_5,'m6=u(32);\n\n');
fprintf(fid_5,'%%Inertia Tensors\n');
fprintf(fid_5,'I111=u(33);\n');
fprintf(fid_5,'I112=u(34);\n');
fprintf(fid_5,'I113=u(35);\n');
fprintf(fid_5,'I122=u(36);\n');
fprintf(fid_5,'I123=u(37);\n');
fprintf(fid_5,'I133=u(38);\n\n');
fprintf(fid_5,'I211=u(39);\n');
fprintf(fid_5,'I212=u(40);\n');
fprintf(fid_5,'I213=u(41);\n');
fprintf(fid_5,'I222=u(42);\n');
fprintf(fid_5,'I223=u(43);\n');
fprintf(fid_5,'I233=u(44);\n\n');
fprintf(fid_5,'I311=u(45);\n');
fprintf(fid_5,'I312=u(46);\n');
fprintf(fid_5,'I313=u(47);\n');
fprintf(fid_5,'I322=u(48);\n');
fprintf(fid_5,'I323=u(49);\n');
fprintf(fid_5,'I333=u(50);\n\n');
fprintf(fid_5,'I411=u(51);\n');
fprintf(fid_5,'I412=u(52);\n');
fprintf(fid_5,'I413=u(53);\n');
fprintf(fid_5,'I422=u(54);\n');
fprintf(fid_5,'I423=u(55);\n');
fprintf(fid_5,'I433=u(56);\n\n');
fprintf(fid_5,'I511=u(57);\n');
fprintf(fid_5,'I512=u(58);\n');
fprintf(fid_5,'I513=u(59);\n');
fprintf(fid_5,'I522=u(60);\n');
fprintf(fid_5,'I523=u(61);\n');
fprintf(fid_5,'I533=u(62);\n\n');
fprintf(fid_5,'I611=u(63);\n');
fprintf(fid_5,'I612=u(63);\n');
fprintf(fid_5,'I613=u(64);\n');
fprintf(fid_5,'I622=u(65);\n');
fprintf(fid_5,'I623=u(66);\n');
fprintf(fid_5,'I633=u(67);\n\n');
fprintf(fid_5,'%%Gravity (magnitude of gravitation)\n');
fprintf(fid_5,'g=u(68);\n\n');
fprintf(fid_5,'%%Time\n');
fprintf(fid_5,'t=u(69);\n\n');
fprintf(fid_5,'%%Viscous Friction Matrix\n');
fprintf(fid_5,'Beta(1,1)=u(70);\n');
fprintf(fid_5,'Beta(2,2)=u(71);\n');
fprintf(fid_5,'Beta(3,3)=u(72);\n');
fprintf(fid_5,'Beta(4,4)=u(73);\n');
fprintf(fid_5,'Beta(5,5)=u(74);\n');
fprintf(fid_5,'Beta(6,6)=u(75);\n\n');
fprintf(fid_5,'%%Gravity Vector (direction of gravity wrt world frame)\n');
fprintf(fid_5,'gx=u(75);\n');
fprintf(fid_5,'gy=u(76);\n');
fprintf(fid_5,'gz=u(77);\n\n');
fprintf(fid_5,'%% external forces (joint efforts)\n');
fprintf(fid_5,'Tao=[u(78);u(79);u(80);u(81);u(82);u(83)];\n\n');
fprintf(fid_5,'%% external forces damped\n');
fprintf(fid_5,'Tao_d = Tao-Beta*pQ;\n\n');
fprintf(fid_5,'%%%% dynamics matrices\n');
fprintf(fid_5,'%% inertia matrix\n');
for i=1:6
    for j=1:6
        fprintf(fid_5,'M(%d,%d)=%s;\n',i,j,char(M(i,j)));
    end
end
fprintf(fid_5,'\n');
fprintf(fid_5,'%% coriolis & centripetal effects matrix\n');
for i=1:6
    for j=1:6
        fprintf(fid_5,'C(%d,%d)=%s;\n',i,j,char(C(i,j)));
    end
end
fprintf(fid_5,'\n');
fprintf(fid_5,'%% gravitational effects matrix\n');
for i=1:6
    fprintf(fid_5,'G(%d,1)=%s;\n',i,char(G(i,1)));
end
fprintf(fid_5,'\n');
fprintf(fid_5,'%%%% compute joint accelerations\n');
fprintf(fid_5,'%% external forces (joint efforts)\n');
fprintf(fid_5,'Tao=[u(46);u(47);u(48)];\n\n');
fprintf(fid_5,'%% joint accelerations\n');
fprintf(fid_5,'ppQ=M\\(Tao_d-C*pQ-G);\n\n');
fprintf(fid_5,'end \n');
fclose(fid_5);

%% model for lyapunov function which will be plotted
% fid_6 = fopen('calcLyapFct.m','w');
% fprintf(fid_6,'function lyap_fct = calcLyapFct(u)\n\n');
% fprintf(fid_6,'%% This file was generated automatically by "symModel_UR10_3".\n\n');
% fprintf(fid_6,'%%%% parameters\n');
% fprintf(fid_5,'%%Joint Position\n');
% fprintf(fid_5,'q1=u(1);\n');
% fprintf(fid_5,'q2=u(2);\n');
% fprintf(fid_5,'q3=u(3);\n\n');
% fprintf(fid_6,'%%Kinematic Parameters\n');
% fprintf(fid_6,'L1=u(7);\n');
% fprintf(fid_6,'L2=u(8);\n');
% fprintf(fid_6,'L3=u(9);\n');
% fprintf(fid_6,'L4=u(10);\n');
% fprintf(fid_6,'L5=u(11);\n');
% fprintf(fid_6,'L6=u(12);\n');
% fprintf(fid_6,'L7=u(13);\n');
% fprintf(fid_6,'L8=u(14);\n');
% fprintf(fid_6,'L9=u(15);\n');
% fprintf(fid_6,'L10=u(16);\n');
% fprintf(fid_6,'L11=L2+L4;\n\n');
% fprintf(fid_6,'%%Dynamic Parameters\n');
% fprintf(fid_6,'m1=u(17);\n');
% fprintf(fid_6,'m2=u(18);\n');
% fprintf(fid_6,'m3=u(19);\n\n');
% fprintf(fid_6,'%%Inertia Tensors\n');
% fprintf(fid_6,'I111=u(20);\n');
% fprintf(fid_6,'I112=u(21);\n');
% fprintf(fid_6,'I113=u(22);\n');
% fprintf(fid_6,'I122=u(23);\n');
% fprintf(fid_6,'I123=u(24);\n');
% fprintf(fid_6,'I133=u(25);\n\n');
% fprintf(fid_6,'I211=u(26);\n');
% fprintf(fid_6,'I212=u(27);\n');
% fprintf(fid_6,'I213=u(28);\n');
% fprintf(fid_6,'I222=u(29);\n');
% fprintf(fid_6,'I223=u(30);\n');
% fprintf(fid_6,'I233=u(31);\n\n');
% fprintf(fid_6,'I311=u(32);\n');
% fprintf(fid_6,'I312=u(33);\n');
% fprintf(fid_6,'I313=u(34);\n');
% fprintf(fid_6,'I322=u(35);\n');
% fprintf(fid_6,'I323=u(36);\n');
% fprintf(fid_6,'I333=u(37);\n\n');
% fprintf(fid_6,'%%Gravity (magnitude of gravitation)\n');
% fprintf(fid_6,'g=u(38);\n\n');
% fprintf(fid_6,'%%Time\n');
% fprintf(fid_6,'t=u(39);\n\n');
% fprintf(fid_6,'%%Viscous Friction Matrix\n');
% fprintf(fid_6,'Beta(1,1)=u(40);\n');
% fprintf(fid_6,'Beta(2,2)=u(41);\n');
% fprintf(fid_6,'Beta(3,3)=u(42);\n\n');
% fprintf(fid_6,'%%Gravity Vector (direction of gravity wrt world frame)\n');
% fprintf(fid_6,'gx=u(43);\n');
% fprintf(fid_6,'gy=u(44);\n');
% fprintf(fid_6,'gz=u(45);\n\n');
% fprintf(fid_6,'%%desired position\n');
% fprintf(fid_6,'q1d=u(46);\n');
% fprintf(fid_6,'q2d=u(47);\n');
% fprintf(fid_6,'q3d=u(48);\n\n');
% fprintf(fid_6,'%%desired position\n');
% fprintf(fid_6,'q1d=u(46);\n');
% fprintf(fid_6,'q2d=u(47);\n');
% fprintf(fid_6,'q3d=u(48);\n\n');
% fprintf(fid_6,'%%D-control constants\n');
% fprintf(fid_6,'Kd=diag([u(49); u(50); u(51)]);\n\n');
% fprintf(fid_6,'%%P-control gains\n');
% fprintf(fid_6,'Kp=diag([u(52); u(53); u(54)]);\n\n');
% fprintf(fid_6,'%%I-control constants\n');
% fprintf(fid_6,'Ki=diag([u(55); u(56); u(57)]);\n\n');
% fprintf(fid_6,'%%Joint Position Vector\n');
% fprintf(fid_6,'Q=[q1; q2; q3];\n\n');
% fprintf(fid_6,'%%Joint Velocity Vector\n');
% fprintf(fid_6,'pQ=[pq1; pq2; pq3];\n\n');
% fprintf(fid_6,'%%Desired Joint Position Vector\n');
% fprintf(fid_6,'Qd=[q1d; q2d; q3d];\n\n');
% fprintf(fid_6,'%%Joint Position Error\n');
% fprintf(fid_6,'DeltaQ=Qd-Q;\n\n');
% fprintf(fid_6,'%%%% dynamics matrices\n');
% fprintf(fid_6,'%% inertia matrix\n');
% for i=1:3
%     for j=1:3
%         fprintf(fid_6,'M(%d,%d)=%s;\n',i,j,char(M(i,j)));
%     end
% end
% fprintf(fid_6,'\n');
% fprintf(fid_6,'%%%% Lyapunov function\n');
% fprintf(fid_6, 'lyap_fct = 0.5*pQ''*M*pQ+0.5*DeltaQ''*Kp*DeltaQ;\n'); 
% fprintf(fid_6, '%%lyap_fct = 0.5*pQ''*M*pQ;\n'); 
% fprintf(fid_6,'end \n');

% Create trajectory according to 5th degree polynomial
fid_7 = fopen('createEvalPTPTraj.m','w');
fprintf(fid_7,'function [Out] = createEvalPTPTraj(t,p_init,p_final,t_init,t_final)\n\n');
fprintf(fid_7,'%% This file was generated automatically by "symModel_UR10_3".\n\n');
for i=1:length(a_x)
    fprintf(fid_7,'a(%d,1) = %s;\n',i,char(a_x(i,1)));
end
fprintf(fid_7,'\n');
fprintf(fid_7,'poly = a(1,1)+a(2,1)*(t-t_init)+a(3,1)*(t-t_init)^2+a(4,1)*(t-t_init)^3+a(5,1)*(t-t_init)^4+a(6,1)*(t-t_init)^5;\n');
fprintf(fid_7,'ppoly = a(2,1)+2*a(3,1)*(t-t_init)+3*a(4,1)*(t-t_init)^2+4*a(5,1)*(t-t_init)^3+5*a(6,1)*(t-t_init)^4;\n');
fprintf(fid_7,'pppoly = 2*a(3,1)+6*a(4,1)*(t-t_init)+12*a(5,1)*(t-t_init)+20*a(6,1)*(t-t_init)^3;\n');
fprintf(fid_7,'p_d_w_1 = (p_final(1,1)-p_init(1,1))*poly+p_init(1,1);\n');
fprintf(fid_7,'p_d_w_2 = (p_final(2,1)-p_init(2,1))*poly+p_init(2,1);\n');
fprintf(fid_7,'p_d_w_3 = (p_final(3,1)-p_init(3,1))*poly+p_init(3,1);\n');
fprintf(fid_7,'pp_d_w_1 = (p_final(1,1)-p_init(1,1))*ppoly;\n');
fprintf(fid_7,'pp_d_w_2 = (p_final(2,1)-p_init(2,1))*ppoly;\n');
fprintf(fid_7,'pp_d_w_3 = (p_final(3,1)-p_init(3,1))*ppoly;\n');
fprintf(fid_7,'ppp_d_w_1 = (p_final(1,1)-p_init(1,1))*pppoly;\n');
fprintf(fid_7,'ppp_d_w_2 = (p_final(2,1)-p_init(2,1))*ppoly;\n');
fprintf(fid_7,'ppp_d_w_3 = (p_final(3,1)-p_init(3,1))*ppoly;\n');
fprintf(fid_7,'\n');
fprintf(fid_7,'Out = [p_d_w_1;p_d_w_2;p_d_w_3;pp_d_w_1;pp_d_w_2;pp_d_w_3;ppp_d_w_1;ppp_d_w_2;ppp_d_w_3];\n');
fprintf(fid_7,'end \n');
fclose(fid_7);

end

