function dqdt = f3(q)
    global M J_1 l_b % body
    global m J_2 l_ab % viscera
    global k_ab b_ab  % stiffness of abdominal oblique muscle
    global g
   
    global k_H b_H L_0H k_F b_F L_0F 
    global xH_toe xF_toe
    global tau_nF tau_nH
    
    param = [M J_1 l_b m J_2 l_ab k_ab b_ab g];
    
    q_pre = q(1:4);
    dq_pre = q(5:8);
    
    %��ʉ���
    %���W
    xb = q_pre(1) - l_b*cos(q_pre(3));
    dxb = dq_pre(1) + l_b*dq_pre(3)*sin(q_pre(3));
    yb = q_pre(2) - l_b*sin(q_pre(3));
    dyb = dq_pre(2) - l_b*dq_pre(3)*cos(q_pre(3));
    phi_H = atan2(yb,(xb-xH_toe));
    
    xf = q_pre(1) + l_b*cos(q_pre(3));
    dxf = dq_pre(1) - l_b*dq_pre(3)*sin(q_pre(3));
    yf = q_pre(2) + l_b*sin(q_pre(3));
    dyf = dq_pre(2) + l_b*dq_pre(3)*cos(q_pre(3));
    phi_F = atan2(yf,(xf-xF_toe));
    
    %�r�̒����Ɨ�
    L_F = sqrt((xf-xF_toe)^2+yf^2);
    F_LF = k_F*(L_0F-L_F) - b_F*((xf-xF_toe)*dxf + yf*dyf)/L_F;
    F_TF = tau_nF/L_F;
    
    L_H = sqrt((xb-xH_toe)^2+yb^2);
    F_LH = k_H*(L_0H-L_H) - b_H*((xb-xH_toe)*dxb + yb*dyb)/L_H;
    F_TH = tau_nH/L_H;
   
    F_F = [F_LF*cos(phi_F) + F_TF*sin(phi_F); F_LF*sin(phi_F) - F_TF*cos(phi_F)];
    tau_F = cross([l_b*cos(q_pre(3)); l_b*sin(q_pre(3));0],[F_F;0]);
    F_H = [F_LH*cos(phi_H)+F_TH*sin(phi_H);F_LH*sin(phi_H)+F_TH*cos(phi_H)];
    tau_H = cross([-l_b*cos(q_pre(3)); -l_b*sin(q_pre(3));0],[F_H;0]);
    
%     tautau = tau_F(3)+tau_H(3)
    Q = [F_F+F_H;tau_F(3)+tau_H(3);0];
    
    %�����s��
    MassMatrix = myMassMatrix(q_pre, param);
    %�R���I���E�d��
    f_cg = myCoriGrav(q_pre, dq_pre, param);
    
    %�����x
    ddq = MassMatrix\(-f_cg+Q);
    
    dqdt = [dq_pre; ddq];