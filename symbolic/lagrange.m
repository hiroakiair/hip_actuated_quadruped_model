clear; format compact   %initialize


syms q [4 1]
syms dq [4 1]
% q = [x_1; y_1; pitch_1; theta_2];
% dq = [dx_1; dy_1; dpitch_1; dtheta_2];


syms M J_1 l_b % body
syms m J_2 l_ab % viscera
syms k_ab b_ab % stiffness of abdominal oblique muscle
syms g 


param = [M J_1 l_b m J_2 l_ab k_ab b_ab g];

%回転行列の定義
% R_1 = [cos(q(3)) -sin(q(3));sin(q(3)) cos(q(3))];
% 
% R_1 = simplify(R_1);

%bodyの位置座標の定義
r_F = [l_b*cos(q(3)); l_b*sin(q(3))] + [q(1); q(2)];
r_H = [-l_b*cos(q(3)); -l_b*sin(q(3))] + [q(1); q(2)];
%bodyの速度
dr_F = [-l_b*dq(3)*sin(q(3)); l_b*dq(3)*cos(q(3))] + [dq(1); dq(2)];
dr_H = [l_b*dq(3)*sin(q(3)); -l_b*dq(3)*cos(q(3))] + [dq(1); dq(2)];

%pendulumの位置速度
rx_2 = q(1) - l_ab*cos(q(4));
ry_2 = q(2) - l_ab*sin(q(4));
drx_2 = dq(1) + l_ab*dq(4)*sin(q(4));
dry_2 = dq(2) - l_ab*dq(4)*cos(q(4));


%ラグランジアンの定義
%散逸項も可能
K1 = 0.5*M*(dq(1)^2+dq(2)^2) + 0.5*J_1*dq(3)^2;
K2 = 0.5*m*(drx_2^2 + dry_2^2) + 0.5*J_2*dq(4)^2;
K = K1 + K2;
U = M*g*q(2) + m*g*ry_2 + 0.5*k_ab*(q(4)-q(3)-pi/2)^2;

L = K-U;

D = 0.5*b_ab*(dq(4)-dq(3))^2;

%matlabFunctionに保存
M_mat = jacobian(jacobian(L,dq),dq);
M_mat = simplify(M_mat)
matlabFunction(M_mat,'File','myMassMatrix','Vars',{q, param});
CoriGrav = jacobian(jacobian(L,dq),q)*dq - jacobian(L,q).';
CoriGrav = simplify(CoriGrav)
matlabFunction(CoriGrav,'File','myCoriGrav','Vars',{q, dq, param});
Dissipation = jacobian(D,dq)
matlabFunction(CoriGrav,'File','myDissipation','Vars',{q, dq, param});

% dx = [dq; M_mat\(-CoriGrav + Q)];